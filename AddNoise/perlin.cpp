#include "perlin.h"

static float fade(float t) {
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

float grad(int hash, float x, float y) {
  switch (hash & 7) {
  case 0:
    return x;
  case 1:
    return -x;
  case 2:
    return y;
  case 3:
    return -y;
  case 4:
    return x + y;
  case 5:
    return -x + y;
  case 6:
    return x - y;
  default:
    return -x - y;
  }
}

static float lerp(float a, float b, float t) { return a + t * (b - a); }

float perlin(int px, int py, float xsize, float ysize, int p[PERM_SIZE]) {
  float x = px / xsize;
  float y = py / ysize;

  int ix = (int)x;
  int iy = (int)y;
  float xf = x - ix;
  float yf = y - iy;

  float u = fade(xf);
  float v = fade(yf);

  iy &= 255;

  int a = p[(ix + p[iy]) % PERM_SIZE];
  int b = p[(ix + p[iy] + 1) % PERM_SIZE];
  int c = p[(ix + p[iy + 1]) % PERM_SIZE];
  int d = p[(ix + p[iy + 1] + 1) % PERM_SIZE];

  float g00 = grad(a, xf, yf);
  float g10 = grad(b, xf - 1, yf);
  float g01 = grad(c, xf, yf - 1);
  float g11 = grad(d, xf - 1, yf - 1);

  float x0 = lerp(g00, g10, u);
  float x1 = lerp(g01, g11, u);

  return lerp(x0, x1, v);
}

template <typename noise_t>
void perlin_create(void *_data, int width, int height, float xsize, float ysize,
                   float scale, int p[PERM_SIZE]) noexcept {
  auto data{reinterpret_cast<noise_t *>(_data)};
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      float r = perlin(x, y, xsize, ysize, p);
      if constexpr (std::is_integral_v<noise_t>) {
        *data++ = static_cast<noise_t>(std::round(r * scale));
      } else {
        *data++ = r * scale;
      }
    }
  }
}

template void perlin_create<int8_t>(void *_data, int width, int height,
                                    float xsize, float ysize, float scale,
                                    int p[PERM_SIZE]) noexcept;
template void perlin_create<int16_t>(void *_data, int width, int height,
                                     float xsize, float ysize, float scale,
                                     int p[PERM_SIZE]) noexcept;
template void perlin_create<float>(void *_data, int width, int height,
                                   float xsize, float ysize, float scale,
                                   int p[PERM_SIZE]) noexcept;
