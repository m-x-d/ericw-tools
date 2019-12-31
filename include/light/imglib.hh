/*  Copyright (C) 1996-1997  Id Software, Inc.
Copyright (C) 2017 Eric Wasylishen

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

#ifndef __COMMON_IMGLIB_H__
#define __COMMON_IMGLIB_H__

#include <common/cmdlib.hh>
#include <common/bspfile.hh>

typedef struct {
    char name[32];
    unsigned width, height;
    unsigned offsets[MIPLEVELS]; // four mip maps stored
    char animname[32]; // next frame in animation chain
    int flags;
    int contents;
    int value;
} q2_miptex_t;

typedef struct {
    byte r;
    byte g;
    byte b;
    byte a;
} color_rgba;

//mxd. Moved from ltface.cc
extern byte thepalette[768];

// Palette
void LoadPalette(bspdata_t *bspdata);
// Returns color components in [0, 255]
qvec3f Palette_GetColor(const int i);
//mxd. Returns RGBA color components in [0, 255]
qvec4f Texture_GetColor(const rgba_miptex_t *tex, const int i, const int rgba_texture_offset = RGBA_TEXTURE_OFFSET);

// Image loading
qboolean LoadPCX(const char *filename, byte **pixels, byte **palette, int *width, int *height);
qboolean LoadTGA(const char *filename, byte **pixels, int *width, int *height);
qboolean LoadWAL(const char *filename, byte **pixels, int *width, int *height);

// Texture loading
void LoadOrConvertTextures(mbsp_t *bsp); // Loads textures from disk and stores them in bsp->drgbatexdata (Quake 2) / Converts paletted bsp->dtexdata textures to RGBA bsp->drgbatexdata textures (Quake / Hexen2)

#endif
