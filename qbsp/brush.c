/*
    Copyright (C) 1996-1997  Id Software, Inc.
    Copyright (C) 1997       Greg Lewis

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

    See file, 'COPYING', for details.
*/

#include <string.h>

#include "qbsp.h"

int numbrushplanes;

int numbrushfaces;
mapface_t faces[128];		// beveled clipping hull can generate many extra


/*
=================
CheckFace

Note: this will not catch 0 area polygons
=================
*/
void
CheckFace(face_t *f)
{
    int i, j;
    vec_t *p1, *p2;
    vec_t d, edgedist;
    vec3_t dir, edgenormal, facenormal;

    if (f->numpoints < 3)
	Message(msgError, errTooFewPoints, f->numpoints);

    VectorCopy(pPlanes[f->planenum].normal, facenormal);
    if (f->planeside) {
	VectorSubtract(vec3_origin, facenormal, facenormal);
    }

    for (i = 0; i < f->numpoints; i++) {
	p1 = f->pts[i];

	for (j = 0; j < 3; j++)
	    if (p1[j] > BOGUS_RANGE || p1[j] < -BOGUS_RANGE)
		Message(msgError, errBogusRange, p1[j]);

	j = i + 1 == f->numpoints ? 0 : i + 1;

	// check the point is on the face plane
	d = DotProduct(p1,
		       pPlanes[f->planenum].normal) -
	    pPlanes[f->planenum].dist;
	if (d < -ON_EPSILON || d > ON_EPSILON)
	    // This used to be an error
	    Message(msgWarning, warnPointOffPlane, p1[0], p1[1], p1[2], d);

	// check the edge isn't degenerate
	p2 = f->pts[j];
	VectorSubtract(p2, p1, dir);

	if (VectorLength(dir) < ON_EPSILON) {
	    Message(msgWarning, warnDegenerateEdge, p1[0], p1[1], p1[2]);
	    for (j = i + 1; j < f->numpoints; j++)
		VectorCopy(f->pts[j], f->pts[j - 1]);
	    f->numpoints--;
	    CheckFace(f);
	    break;
	}

	CrossProduct(facenormal, dir, edgenormal);
	VectorNormalize(edgenormal);
	edgedist = DotProduct(p1, edgenormal);
	edgedist += ON_EPSILON;

	// all other points must be on front side
	for (j = 0; j < f->numpoints; j++) {
	    if (j == i)
		continue;
	    d = DotProduct(f->pts[j], edgenormal);
	    if (d > edgedist)
		Message(msgError, errConcaveFace);
	}
    }
}


//===========================================================================

/*
=================
AddToBounds
=================
*/
void
AddToBounds(vec3_t v)
{
    int i;

    for (i = 0; i < 3; i++) {
	if (v[i] < pCurEnt->mins[i])
	    pCurEnt->mins[i] = v[i];
	if (v[i] > pCurEnt->maxs[i])
	    pCurEnt->maxs[i] = v[i];
    }
}

//===========================================================================

int
PlaneTypeForNormal(vec3_t normal)
{
    float ax, ay, az;

    // NOTE: should these have an epsilon around 1.0?
    if (normal[0] == 1.0)
	return PLANE_X;
    if (normal[1] == 1.0)
	return PLANE_Y;
    if (normal[2] == 1.0)
	return PLANE_Z;

    if (normal[0] == -1.0 || normal[1] == -1.0 || normal[2] == -1.0)
	Message(msgError, errNonCanonicalVector);

    ax = fabs(normal[0]);
    ay = fabs(normal[1]);
    az = fabs(normal[2]);

    if (ax >= ay && ax >= az)
	return PLANE_ANYX;
    if (ay >= ax && ay >= az)
	return PLANE_ANYY;
    return PLANE_ANYZ;
}

#define	DISTEPSILON		0.01
#define	ANGLEEPSILON	0.00001

void
NormalizePlane(plane_t *dp)
{
    vec_t ax, ay, az;

    if (dp->normal[0] == -1.0) {
	dp->normal[0] = 1.0;
	dp->dist = -dp->dist;
    } else if (dp->normal[1] == -1.0) {
	dp->normal[1] = 1.0;
	dp->dist = -dp->dist;
    } else if (dp->normal[2] == -1.0) {
	dp->normal[2] = 1.0;
	dp->dist = -dp->dist;
    }

    if (dp->normal[0] == 1.0) {
	dp->type = PLANE_X;
	return;
    }
    if (dp->normal[1] == 1.0) {
	dp->type = PLANE_Y;
	return;
    }
    if (dp->normal[2] == 1.0) {
	dp->type = PLANE_Z;
	return;
    }

    ax = fabs(dp->normal[0]);
    ay = fabs(dp->normal[1]);
    az = fabs(dp->normal[2]);

    if (ax >= ay && ax >= az)
	dp->type = PLANE_ANYX;
    else if (ay >= ax && ay >= az)
	dp->type = PLANE_ANYY;
    else
	dp->type = PLANE_ANYZ;
    if (dp->normal[dp->type - PLANE_ANYX] < 0) {
	VectorSubtract(vec3_origin, dp->normal, dp->normal);
	dp->dist = -dp->dist;
    }
}

/*
===============
FindPlane

Returns a global plane number and the side that will be the front
===============
*/
int
FindPlane(plane_t *dplane, int *side)
{
    int i;
    plane_t *dp, pl;
    vec_t dot;

    pl = *dplane;
    NormalizePlane(&pl);
    if (DotProduct(pl.normal, dplane->normal) > 0)
	*side = 0;
    else
	*side = 1;

    // Find this plane if it already exists
    dp = pPlanes;
    for (i = 0; i < numbrushplanes; i++, dp++) {
	dot = DotProduct(dp->normal, pl.normal);
	if (dot > 1.0 - ANGLEEPSILON &&
	    fabs(dp->dist - pl.dist) < DISTEPSILON) {
	    return i;
	}
    }

    if (numbrushplanes >= cPlanes)
	Message(msgError, errLowBrushPlaneCount);

    dot = VectorLength(dplane->normal);
    if (dot < 1.0 - ANGLEEPSILON || dot > 1.0 + ANGLEEPSILON)
	Message(msgError, errNormalization, dot);

    pPlanes[numbrushplanes] = pl;
    numbrushplanes++;

    return numbrushplanes - 1;
}


/*
=============================================================================

			TURN BRUSHES INTO GROUPS OF FACES

=============================================================================
*/

vec3_t brush_mins, brush_maxs;

/*
=================
FindTargetEntity
=================
*/
int
FindTargetEntity(char *szTarget)
{
    int iEntity;
    char *szName;

    for (iEntity = 0; iEntity < map.cEntities; iEntity++) {
	szName = ValueForKey(iEntity, "targetname");
	if (szName && !strcasecmp(szTarget, szName))
	    return iEntity;
    }

    return -1;
}


/*
=================
FixRotateOrigin
=================
*/
void
FixRotateOrigin(int iEntity, vec3_t offset)
{
    int iFoundEnt;
    char *szSearch;
    char szOrigin[20];

    szSearch = ValueForKey(iEntity, "target");
    if (!szSearch) {
	szSearch = ValueForKey(iEntity, "classname");
	Message(msgWarning, warnNoRotateTarget, szSearch);
    } else {
	iFoundEnt = FindTargetEntity(szSearch);
	if (iFoundEnt != -1)
	    GetVectorForKey(iFoundEnt, "origin", offset);
    }
    sprintf(szOrigin, "%d %d %d", (int)offset[0], (int)offset[1],
	    (int)offset[2]);
    SetKeyValue(iEntity, "origin", szOrigin);
}


/*
=================
CreateBrushFaces
=================
*/
#define	ZERO_EPSILON	0.001
face_t *
CreateBrushFaces(void)
{
    int i, j, k;
    vec_t r;
    face_t *f;
    winding_t *w;
    plane_t plane;
    face_t *pFaceList = NULL;
    mapface_t *pFace;
    vec3_t offset;
    char *szClassname;
    vec3_t point;
    vec_t max, min;

    offset[0] = offset[1] = offset[2] = 0;
    min = brush_mins[0] = brush_mins[1] = brush_mins[2] = 99999;
    max = brush_maxs[0] = brush_maxs[1] = brush_maxs[2] = -99999;

    // Hipnotic rotation
    szClassname = ValueForKey(map.iEntities, "classname");
    if (!strncmp(szClassname, "rotate_", 7))
	FixRotateOrigin(map.iEntities, offset);

    for (i = 0; i < numbrushfaces; i++) {
	pFace = &faces[i];

	w = BaseWindingForPlane(&pFace->plane);

	for (j = 0; j < numbrushfaces && w; j++) {
	    if (j == i)
		continue;
	    // flip the plane, because we want to keep the back side
	    VectorSubtract(vec3_origin, faces[j].plane.normal, plane.normal);
	    plane.dist = -faces[j].plane.dist;

	    w = ClipWinding(w, &plane, false);
	}

	if (!w)
	    continue;		// overcontrained plane

	// this face is a keeper
	f = (face_t *)AllocMem(FACE, 1, true);
	f->numpoints = w->numpoints;
	if (f->numpoints > MAXEDGES)
	    Message(msgError, errLowFacePointCount);

	for (j = 0; j < w->numpoints; j++) {
	    for (k = 0; k < 3; k++) {
		point[k] = w->points[j][k] - offset[k];
		r = Q_rint(point[k]);
		if (fabs(point[k] - r) < ZERO_EPSILON)
		    f->pts[j][k] = r;
		else
		    f->pts[j][k] = point[k];

		if (f->pts[j][k] < brush_mins[k])
		    brush_mins[k] = f->pts[j][k];
		if (f->pts[j][k] > brush_maxs[k])
		    brush_maxs[k] = f->pts[j][k];
		if (f->pts[j][k] < min)
		    min = f->pts[j][k];
		if (f->pts[j][k] > max)
		    max = f->pts[j][k];
	    }
	}

	VectorCopy(pFace->plane.normal, plane.normal);
	VectorScale(pFace->plane.normal, pFace->plane.dist, point);
	VectorSubtract(point, offset, point);
	plane.dist = DotProduct(plane.normal, point);

	FreeMem(w, WINDING, 1);

	f->texturenum = hullnum ? 0 : pFace->texinfo;
	f->planenum = FindPlane(&plane, &f->planeside);
	f->next = pFaceList;
	pFaceList = f;
	CheckFace(f);
    }

    // Rotatable objects must have a bounding box big enough to
    // account for all its rotations
    if (!strncmp(szClassname, "rotate_", 7)) {
	vec_t delta;

	delta = fabs(max);
	if (fabs(min) > delta)
	    delta = fabs(min);

	for (k = 0; k < 3; k++) {
	    brush_mins[k] = -delta;
	    brush_maxs[k] = delta;
	}
    }

    return pFaceList;
}


/*
=================
FreeBrushFaces
=================
*/
void
FreeBrushFaces(face_t *pFaceList)
{
    face_t *pFace, *pNext;

    for (pFace = pFaceList; pFace; pFace = pNext) {
	pNext = pFace->next;
	FreeMem(pFace, FACE, 1);
    }
}


/*
=====================
FreeBrushsetBrushes
=====================
*/
void
FreeBrushsetBrushes(void)
{
    brush_t *pBrush, *pNext;

    for (pBrush = pCurEnt->pBrushes; pBrush; pBrush = pNext) {
	pNext = pBrush->next;
	FreeBrushFaces(pBrush->faces);
	FreeMem(pBrush, BRUSH, 1);
    }
}


/*
==============================================================================

BEVELED CLIPPING HULL GENERATION

This is done by brute force, and could easily get a lot faster if anyone cares.
==============================================================================
*/

// TODO: fix this whole thing
#define	MAX_HULL_POINTS	64	//32
#define	MAX_HULL_EDGES	128	//64

int num_hull_points;
vec3_t hull_points[MAX_HULL_POINTS];
vec3_t hull_corners[MAX_HULL_POINTS * 8];
int num_hull_edges;
int hull_edges[MAX_HULL_EDGES][2];

/*
============
AddBrushPlane
=============
*/
void
AddBrushPlane(plane_t *plane)
{
    int i;
    plane_t *pl;
    float l;

    l = VectorLength(plane->normal);
    if (l < 0.999 || l > 1.001)
	Message(msgError, errInvalidNormal, l);

    for (i = 0; i < numbrushfaces; i++) {
	pl = &faces[i].plane;
	if (VectorCompare(pl->normal, plane->normal) &&
	    fabs(pl->dist - plane->dist) < ON_EPSILON)
	    return;
    }
    if (numbrushfaces >= MAX_FACES)
	Message(msgError, errLowBrushFaceCount);
    faces[i].plane = *plane;
    faces[i].texinfo = 0;
    numbrushfaces++;
}


/*
============
TestAddPlane

Adds the given plane to the brush description if all of the original brush
vertexes can be put on the front side
=============
*/
void
TestAddPlane(plane_t *plane)
{
    int i, c;
    vec_t d;
    vec_t *corner;
    plane_t flip;
    vec3_t inv;
    int counts[3];
    plane_t *pl;

    // see if the plane has already been added
    for (i = 0; i < numbrushfaces; i++) {
	pl = &faces[i].plane;
	if (VectorCompare(plane->normal, pl->normal)
	    && fabs(plane->dist - pl->dist) < ON_EPSILON)
	    return;
	VectorSubtract(vec3_origin, plane->normal, inv);
	if (VectorCompare(inv, pl->normal)
	    && fabs(plane->dist + pl->dist) < ON_EPSILON)
	    return;
    }

    // check all the corner points
    counts[0] = counts[1] = counts[2] = 0;
    c = num_hull_points * 8;

    corner = hull_corners[0];
    for (i = 0; i < c; i++, corner += 3) {
	d = DotProduct(corner, plane->normal) - plane->dist;
	if (d < -ON_EPSILON) {
	    if (counts[0])
		return;
	    counts[1]++;
	} else if (d > ON_EPSILON) {
	    if (counts[1])
		return;
	    counts[0]++;
	} else
	    counts[2]++;
    }

    // the plane is a seperator
    if (counts[0]) {
	VectorSubtract(vec3_origin, plane->normal, flip.normal);
	flip.dist = -plane->dist;
	plane = &flip;
    }

    AddBrushPlane(plane);
}

/*
============
AddHullPoint

Doesn't add if duplicated
=============
*/
int
AddHullPoint(vec3_t p, vec3_t hull_size[2])
{
    int i;
    vec_t *c;
    int x, y, z;

    for (i = 0; i < num_hull_points; i++)
	if (VectorCompare(p, hull_points[i]))
	    return i;

    if (num_hull_points == MAX_HULL_POINTS)
	Message(msgError, errLowHullPointCount);

    VectorCopy(p, hull_points[num_hull_points]);

    c = hull_corners[i * 8];

    for (x = 0; x < 2; x++)
	for (y = 0; y < 2; y++)
	    for (z = 0; z < 2; z++) {
		c[0] = p[0] + hull_size[x][0];
		c[1] = p[1] + hull_size[y][1];
		c[2] = p[2] + hull_size[z][2];
		c += 3;
	    }

    num_hull_points++;

    return i;
}


/*
============
AddHullEdge

Creates all of the hull planes around the given edge, if not done allready
=============
*/
void
AddHullEdge(vec3_t p1, vec3_t p2, vec3_t hull_size[2])
{
    int pt1, pt2;
    int i;
    int a, b, c, d, e;
    vec3_t edgevec, planeorg, planevec;
    plane_t plane;
    vec_t l;

    pt1 = AddHullPoint(p1, hull_size);
    pt2 = AddHullPoint(p2, hull_size);

    for (i = 0; i < num_hull_edges; i++)
	if ((hull_edges[i][0] == pt1 && hull_edges[i][1] == pt2)
	    || (hull_edges[i][0] == pt2 && hull_edges[i][1] == pt1))
	    return;		// allread added

    if (num_hull_edges == MAX_HULL_EDGES)
	Message(msgError, errLowHullEdgeCount);

    hull_edges[i][0] = pt1;
    hull_edges[i][1] = pt2;
    num_hull_edges++;

    VectorSubtract(p1, p2, edgevec);
    VectorNormalize(edgevec);

    for (a = 0; a < 3; a++) {
	b = (a + 1) % 3;
	c = (a + 2) % 3;
	for (d = 0; d <= 1; d++)
	    for (e = 0; e <= 1; e++) {
		VectorCopy(p1, planeorg);
		planeorg[b] += hull_size[d][b];
		planeorg[c] += hull_size[e][c];

		VectorCopy(vec3_origin, planevec);
		planevec[a] = 1;

		CrossProduct(planevec, edgevec, plane.normal);
		l = VectorLength(plane.normal);
		if (l < 1 - ANGLEEPSILON || l > 1 + ANGLEEPSILON)
		    continue;
		plane.dist = DotProduct(planeorg, plane.normal);

		TestAddPlane(&plane);
	    }
    }
}


/*
============
ExpandBrush
=============
*/
void
ExpandBrush(vec3_t hull_size[2], face_t *pFaceList)
{
    int i, x, s;
    vec3_t corner;
    face_t *f;
    plane_t plane, *p;
    int cBevEdge = 0;

    num_hull_points = 0;
    num_hull_edges = 0;

    // create all the hull points
    for (f = pFaceList; f; f = f->next)
	for (i = 0; i < f->numpoints; i++) {
	    AddHullPoint(f->pts[i], hull_size);
	    cBevEdge++;
	}

    // expand all of the planes
    for (i = 0; i < numbrushfaces; i++) {
	p = &faces[i].plane;
	VectorCopy(vec3_origin, corner);
	for (x = 0; x < 3; x++) {
	    if (p->normal[x] > 0)
		corner[x] = hull_size[1][x];
	    else if (p->normal[x] < 0)
		corner[x] = hull_size[0][x];
	}
	p->dist += DotProduct(corner, p->normal);
    }

    // add any axis planes not contained in the brush to bevel off corners
    for (x = 0; x < 3; x++)
	for (s = -1; s <= 1; s += 2) {
	    // add the plane
	    VectorCopy(vec3_origin, plane.normal);
	    plane.normal[x] = (float)s;
	    if (s == -1)
		plane.dist = -brush_mins[x] + -hull_size[0][x];
	    else
		plane.dist = brush_maxs[x] + hull_size[1][x];
	    AddBrushPlane(&plane);
	}

    // add all of the edge bevels
    for (f = pFaceList; f; f = f->next)
	for (i = 0; i < f->numpoints; i++)
	    AddHullEdge(f->pts[i], f->pts[(i + 1) % f->numpoints], hull_size);
}

//============================================================================


/*
===============
LoadBrush

Converts a mapbrush to a bsp brush
===============
*/
brush_t *
LoadBrush(int iBrush)
{
    brush_t *b;
    int contents;
    char *szName;
    face_t *pFaceList;

    // check texture name for attributes
    szName =
	rgszMiptex[pWorldEnt->
		   pTexinfo[map.rgFaces[map.rgBrushes[iBrush].iFaceStart].
			    texinfo].miptex];

    if (!strcasecmp(szName, "clip") && hullnum == 0)
	return NULL;		// "clip" brushes don't show up in the draw hull

    // entities never use water merging
    if (map.iEntities != 0)
	contents = CONTENTS_SOLID;
    else if (szName[0] == '*') {
	if (!strncasecmp(szName + 1, "lava", 4))
	    contents = CONTENTS_LAVA;
	else if (!strncasecmp(szName + 1, "slime", 5))
	    contents = CONTENTS_SLIME;
	else
	    contents = CONTENTS_WATER;
    } else if (!strncasecmp(szName, "sky", 3) && hullnum == 0)
	contents = CONTENTS_SKY;
    else
	contents = CONTENTS_SOLID;

    if (hullnum && contents != CONTENTS_SOLID && contents != CONTENTS_SKY)
	return NULL;		// water brushes don't show up in clipping hulls

// no seperate textures on clip hull

    // create the faces
    numbrushfaces =
	map.rgBrushes[iBrush].iFaceEnd - map.rgBrushes[iBrush].iFaceStart;
    memcpy(faces, &(map.rgFaces[map.rgBrushes[iBrush].iFaceStart]),
	   numbrushfaces * sizeof(mapface_t));

    pFaceList = CreateBrushFaces();

    if (!pFaceList) {
	Message(msgWarning, warnNoBrushFaces);
	return NULL;
    }

    if (hullnum == 1) {
	vec3_t size[2] = { {-16, -16, -32}, {16, 16, 24} };

	ExpandBrush(size, pFaceList);
	FreeBrushFaces(pFaceList);
	pFaceList = CreateBrushFaces();
    } else if (hullnum == 2) {
	vec3_t size[2] = { {-32, -32, -64}, {32, 32, 24} };

	ExpandBrush(size, pFaceList);
	FreeBrushFaces(pFaceList);
	pFaceList = CreateBrushFaces();
    }

    // create the brush
    b = (brush_t *)AllocMem(BRUSH, 1, true);

    b->contents = contents;
    b->faces = pFaceList;
    VectorCopy(brush_mins, b->mins);
    VectorCopy(brush_maxs, b->maxs);

    return b;
}

//=============================================================================


/*
============
Brush_LoadEntity
============
*/
void
Brush_LoadEntity(void)
{
    brush_t *b, *next, *water, *other;
    int iBrush, cMapBrushes;
    int i;

    for (i = 0; i < 3; i++) {
	pCurEnt->mins[i] = 99999;
	pCurEnt->maxs[i] = -99999;
    }

    pCurEnt->cBrushes = 0;
    other = water = NULL;

    Message(msgProgress, "Brush_LoadEntity");

    cMapBrushes = pCurEnt->iBrushEnd - pCurEnt->iBrushStart;
    for (iBrush = pCurEnt->iBrushStart; iBrush < pCurEnt->iBrushEnd; iBrush++) {
	b = LoadBrush(iBrush);
	if (!b)
	    continue;

	pCurEnt->cBrushes++;
	Message(msgPercent, pCurEnt->cBrushes, cMapBrushes);

	if (b->contents != CONTENTS_SOLID) {
	    b->next = water;
	    water = b;
	} else {
	    b->next = other;
	    other = b;
	}

	AddToBounds(b->mins);
	AddToBounds(b->maxs);
    }

    // add all of the water textures at the start
    for (b = water; b; b = next) {
	next = b->next;
	b->next = other;
	other = b;
    }

    Message(msgStat, "%5i brushes", pCurEnt->cBrushes);

    // Store the brushes away
    pCurEnt->pBrushes = other;
}
