#pragma once
#include "perlin.h"

template <typename noise_t>
void simplex_create(void *_data, int width, int height, float xsize,
                    float ysize, float scale, int p[PERM_SIZE]) noexcept;

template <typename noise_t>
void fractal_create(void *_data, int width, int height, float xsize,
                    float ysize, float scale, int p[PERM_SIZE], int octaves,
                    float freq, float amp, float lac, float pers) noexcept;
