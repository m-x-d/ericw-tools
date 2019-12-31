/*  Copyright (C) 1996-1997  Id Software, Inc.
Copyright (C) 2018 MaxED

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

#include <cassert>
#include <cstdio>
#include <iostream>

#include <light/light.hh>
#include <light/bounce.hh>
#include <light/surflight.hh>
#include <light/ltface.hh>

#include <common/polylib.hh>
#include <common/bsputils.hh>

#include <vector>
#include <map>
#include <mutex>
#include <string>

#include <common/qvec.hh>

using namespace std;
using namespace polylib;

mutex surfacelights_lock;
std::vector<surfacelight_t> surfacelights;
std::map<int, std::vector<int>> surfacelightsByFacenum;
int total_surflight_points = 0;

struct make_surface_lights_args_t {
    const mbsp_t *bsp;
    const globalconfig_t *cfg;
};

struct save_winding_points_args_t {
    vector<qvec3f> *points;
};

static void 
SaveWindingCenterFn(winding_t *w, void *userinfo)
{
    auto *args = static_cast<save_winding_points_args_t *>(userinfo);

    vec3_t center{};
    WindingCenter(w, center);
    args->points->push_back(vec3_t_to_glm(center));
}

static void
ProcessGlowPoints(const mbsp_t *bsp, const bsp2_dface_t *face, vector<qvec3f> *points, vector<qvec3f> *pointcolors, vec3_t texturecolor)
{
    vector<qvec3f> litpoints;
    qvec3f totalcolor{};
    
    // Discard points, which correspond to nearly-black pixels on glow texture, calculate colors for the ones, which aren't
    for (const qvec3f qp : *points) {
        // Get glow color at this point...
        vec3_t point;
        glm_to_vec3_t(qp, point);
        const color_rgba glowcolor = SampleTexture(face, bsp, point, RGBA_GLOW_OFFSET); // TODO: retrieving the same texture for each point is a bit wasteful

        // Skip when glow texture pixel is nearly black
        if ((glowcolor.r + glowcolor.g + glowcolor.b) / 3 < 25)
            continue;

        // Store point color...
        qvec3f pc = { (float)glowcolor.r, (float)glowcolor.g, (float)glowcolor.b };
        pc /= 255.0f; // Convert to [0 .. 1] range
        pointcolors->push_back(pc);
        totalcolor += pc;

        // This point stays
        litpoints.push_back(qp);
    }

    // Calculate average texture color from pointcolors
    totalcolor /= litpoints.size();
    glm_to_vec3_t(totalcolor, texturecolor);

    // Store processed points
    *points = litpoints;
}

static void *
MakeSurfaceLightsThread(void *arg)
{
    const mbsp_t *bsp = static_cast<make_surface_lights_args_t *>(arg)->bsp;
    const globalconfig_t &cfg = *static_cast<make_surface_lights_args_t *>(arg)->cfg;

    while (true) {
        const int i = GetThreadWork();
        if (i == -1) break;

        const bsp2_dface_t *face = BSP_GetFace(bsp, i);

        // Face casts light?
        const gtexinfo_t *info = Face_Texinfo(bsp, face);
        if (info == nullptr) continue;
        if (!(info->flags & Q2_SURF_LIGHT) || info->value == 0) {
            if (info->flags & Q2_SURF_LIGHT) {
                vec3_t wc;
                WindingCenter(WindingFromFace(bsp, face), wc);
                logprint("WARNING: surface light '%s' at [%s] has 0 intensity.\n", Face_TextureName(bsp, face), VecStr(wc).c_str());
            }
            continue;
        }

        // Create face points...
        auto poly = GLM_FacePoints(bsp, face);
        const float facearea = GLM_PolyArea(poly);
        
        // Avoid small, or zero-area faces
        if(GLM_PolyArea(poly) < 1) continue;

        // Create winding...
        const int numpoints = poly.size();
        winding_t *winding = AllocWinding(numpoints);
        for (int c = 0; c < numpoints; c++) 
            glm_to_vec3_t(poly.at(c), winding->p[c]);
        winding->numpoints = numpoints;
        RemoveColinearPoints(winding);

        // Get face normal and midpoint...
        vec3_t facenormal, facemidpoint;
        Face_Normal(bsp, face, facenormal);
        WindingCenter(winding, facemidpoint);
        VectorMA(facemidpoint, 1, facenormal, facemidpoint); // Lift 1 unit

        // Check glow texture...
        const auto *mt = Face_Miptex(bsp, face);
        const bool haveglowtexture = (cfg.surflightglowtextures.boolValue() && mt->offsets[RGBA_GLOW_OFFSET]);
        const float subdiv = (haveglowtexture ? 1.0f : cfg.surflightsubdivision.floatValue());

        // Dice winding...
        vector<qvec3f> points;
        vector<qvec3f> pointcolors;
        save_winding_points_args_t args{};
        args.points = &points;
        DiceWinding(winding, subdiv, SaveWindingCenterFn, &args);
        winding = nullptr; // DiceWinding frees winding

        vec3_t texturecolor;
        if (haveglowtexture) {
            // Process glow texture, get average texture color from glowing area
            ProcessGlowPoints(bsp, face, &points, &pointcolors, texturecolor);

            // Got black glow texture? WHY?
            if(points.empty())
                continue;
        } else {
            // Get texture color
            Face_LookupTextureColor(bsp, face, texturecolor);

            // Convert to [0..1] range...
            VectorScale(texturecolor, 1.0f / 255.0f, texturecolor); 
        }

        total_surflight_points += points.size();

        // Calculate emit color and intensity...
        VectorScale(texturecolor, info->value, texturecolor); // Scale by light value

        // Calculate intensity...
        float intensity = 0.0f;
        for (float c : texturecolor)
            intensity = max(c, intensity);
        if (intensity == 0.0f) continue;

        // Normalize color...
        if (intensity > 1.0f) VectorScale(texturecolor, 1.0f / intensity, texturecolor);

        // Copy texture color to all points?
        if (!haveglowtexture) {
            for (unsigned c = 0; c < points.size(); c++)
                pointcolors.push_back(vec3_t_to_glm(texturecolor));
        }

        // Sanity checks...
        Q_assert(!points.empty());
        Q_assert(!pointcolors.empty());
        Q_assert(points.size() == pointcolors.size());

        // Add surfacelight...
        surfacelight_t l;
        l.surfnormal = vec3_t_to_glm(facenormal);
        l.points = points;
        l.pointcolors = pointcolors;
        VectorCopy(facemidpoint, l.pos);

        // Store surfacelight settings...
        l.totalintensity = intensity * facearea;
        l.intensity = l.totalintensity / points.size();
        VectorCopy(texturecolor, l.color);

        // Init bbox...
        VectorSet(l.mins, 0, 0, 0);
        VectorSet(l.maxs, 0, 0, 0);

        if (!novisapprox)
            EstimateVisibleBoundsAtPoint(facemidpoint, l.mins, l.maxs);

        // Store light...
        unique_lock<mutex> lck{ surfacelights_lock };
        surfacelights.push_back(l);

        const int index = static_cast<int>(surfacelights.size()) - 1;
        surfacelightsByFacenum[Face_GetNum(bsp, face)].push_back(index);
    }

    return nullptr;
}

const std::vector<surfacelight_t> &SurfaceLights()
{
    return surfacelights;
}

int TotalSurfacelightPoints()
{
    return total_surflight_points;
}

// No surflight_debug (yet?), so unused...
const std::vector<int> &SurfaceLightsForFaceNum(int facenum)
{
    const auto &vec = surfacelightsByFacenum.find(facenum);
    if (vec != surfacelightsByFacenum.end()) 
        return vec->second;

    static std::vector<int> empty;
    return empty;
}

void // Quake 2 surface lights
MakeSurfaceLights(const globalconfig_t &cfg, const mbsp_t *bsp)
{
    logprint("--- MakeSurfaceLights ---\n");

    make_surface_lights_args_t args { bsp,  &cfg };
    RunThreadsOn(0, bsp->numfaces, MakeSurfaceLightsThread, static_cast<void *>(&args));
}
