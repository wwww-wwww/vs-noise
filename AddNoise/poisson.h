#pragma once

#include <algorithm>
#include <random>

template <typename pixel_t>
void poisson_apply(const void *_srcp, void *_dstp,
                   std::default_random_engine &generator, const int width,
                   const int height, const ptrdiff_t stride, int peak,
                   float scale) noexcept {
  auto srcp{reinterpret_cast<const pixel_t *>(_srcp)};
  auto dstp{reinterpret_cast<pixel_t *>(_dstp)};

  for (auto y{0}; y < height; y++) {
    for (auto x{0}; x < width; x++) {
      auto val = srcp[x] / scale;
      std::poisson_distribution<int> distribution(val);
      dstp[x] = static_cast<pixel_t>(
          std::clamp((int)(distribution(generator) * scale), 0, peak));
    }

    srcp += stride;
    dstp += stride;
  }
};
