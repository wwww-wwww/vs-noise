#pragma once

#include <cmath>
#include <vector>

// permutation table size
static constexpr int PERM_SIZE = 256;

float grad(int hash, float x, float y);

template <typename noise_t>
void perlin_create(void *_data, int width, int height, float xsize, float ysize,
                   float scale, int p[PERM_SIZE]) noexcept;
