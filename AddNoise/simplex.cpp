#include "simplex.h"

// https://github.com/SRombauts/SimplexNoise/blob/master/src/SimplexNoise.cpp
float simplex(float px, float py, float xsize, float ysize, int p[PERM_SIZE]) {
  float x = px / xsize;
  float y = py / ysize;

  float n0, n1, n2; // Noise contributions from the three corners

  // Skewing/Unskewing factors for 2D
  static const float F2 = 0.366025403f; // F2 = (sqrt(3) - 1) / 2
  static const float G2 = 0.211324865f; // G2 = (3 - sqrt(3)) / 6

  const float s = (x + y) * F2; // Hairy factor for 2D
  const float xs = x + s;
  const float ys = y + s;
  int i = (int)xs;
  int j = (int)ys;

  const float t = static_cast<float>(i + j) * G2;
  const float X0 = i - t;
  const float Y0 = j - t;
  const float x0 = x - X0; // The x,y distances from the cell origin
  const float y0 = y - Y0;

  int i1, j1; // Offsets for second (middle) corner of simplex in (i,j) coords
  if (x0 > y0) { // lower triangle, XY order: (0,0)->(1,0)->(1,1)
    i1 = 1;
    j1 = 0;
  } else { // upper triangle, YX order: (0,0)->(0,1)->(1,1)
    i1 = 0;
    j1 = 1;
  }

  // A step of (1,0) in (i,j) means a step of (1-c,-c) in (x,y), and
  // a step of (0,1) in (i,j) means a step of (-c,1-c) in (x,y), where
  // c = (3-sqrt(3))/6

  const float x1 = x0 - i1 + G2; // Middle corner unskewed coords
  const float y1 = y0 - j1 + G2;
  const float x2 = x0 - 1.0f + 2.0f * G2; // Last corner unskewed coords
  const float y2 = y0 - 1.0f + 2.0f * G2;

  // Work out the hashed gradient indices of the three simplex corners
  const int gi0 = p[(i + p[j % PERM_SIZE]) % PERM_SIZE];
  const int gi1 = p[(i + i1 + p[(j + j1) % PERM_SIZE]) % PERM_SIZE];
  const int gi2 = p[(i + 1 + p[(j + 1) % PERM_SIZE]) % PERM_SIZE];

  // Calculate the contribution from the first corner
  float t0 = 0.5f - x0 * x0 - y0 * y0;
  if (t0 < 0.0f) {
    n0 = 0.0f;
  } else {
    t0 *= t0;
    n0 = t0 * t0 * grad(gi0, x0, y0);
  }

  // Calculate the contribution from the second corner
  float t1 = 0.5f - x1 * x1 - y1 * y1;
  if (t1 < 0.0f) {
    n1 = 0.0f;
  } else {
    t1 *= t1;
    n1 = t1 * t1 * grad(gi1, x1, y1);
  }

  // Calculate the contribution from the third corner
  float t2 = 0.5f - x2 * x2 - y2 * y2;
  if (t2 < 0.0f) {
    n2 = 0.0f;
  } else {
    t2 *= t2;
    n2 = t2 * t2 * grad(gi2, x2, y2);
  }

  // Add contributions from each corner to get the final noise value.
  // The result is scaled to return values in the interval [-1,1].
  return 45.23065f * (n0 + n1 + n2);
}

template <typename noise_t>
void simplex_create(void *_data, int width, int height, float xsize,
                    float ysize, float scale, int p[PERM_SIZE]) noexcept {
  auto data{reinterpret_cast<noise_t *>(_data)};
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      float r = simplex((float)x, (float)y, xsize, ysize, p);
      if constexpr (std::is_integral_v<noise_t>) {
        *data++ = static_cast<noise_t>(std::round(r * scale));
      } else {
        *data++ = r * scale;
      }
    }
  }
}

template void simplex_create<int8_t>(void *_data, int width, int height,
                                     float xsize, float ysize, float scale,
                                     int p[PERM_SIZE]) noexcept;
template void simplex_create<int16_t>(void *_data, int width, int height,
                                      float xsize, float ysize, float scale,
                                      int p[PERM_SIZE]) noexcept;
template void simplex_create<float>(void *_data, int width, int height,
                                    float xsize, float ysize, float scale,
                                    int p[PERM_SIZE]) noexcept;

// fBm
float fractal(int octaves, int x, int y, float freq, float amp, float lac,
              float pers, float xsize, float ysize, int p[PERM_SIZE]) {
  float output = 0.f;
  float denom = 0.f;

  for (int i = 0; i < octaves; i++) {
    output += (amp * simplex(x * freq, y * freq, xsize, ysize, p));
    denom += amp;

    freq *= lac;
    amp *= pers;
  }

  return output / denom;
}

template <typename noise_t>
void fractal_create(void *_data, int width, int height, float xsize,
                    float ysize, float scale, int p[PERM_SIZE], int octaves,
                    float freq, float amp, float lac, float pers) noexcept {
  auto data{reinterpret_cast<noise_t *>(_data)};
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      float r = fractal(octaves, x, y, freq, amp, lac, pers, xsize, ysize, p);
      if constexpr (std::is_integral_v<noise_t>) {
        *data++ = static_cast<noise_t>(std::round(r * scale));
      } else {
        *data++ = r * scale;
      }
    }
  }
}

template void fractal_create<int8_t>(void *_data, int width, int height,
                                     float xsize, float ysize, float scale,
                                     int p[PERM_SIZE], int octaves, float freq,
                                     float amp, float lac, float pers) noexcept;
template void fractal_create<int16_t>(void *_data, int width, int height,
                                      float xsize, float ysize, float scale,
                                      int p[PERM_SIZE], int octaves, float freq,
                                      float amp, float lac,
                                      float pers) noexcept;
template void fractal_create<float>(void *_data, int width, int height,
                                    float xsize, float ysize, float scale,
                                    int p[PERM_SIZE], int octaves, float freq,
                                    float amp, float lac, float pers) noexcept;
