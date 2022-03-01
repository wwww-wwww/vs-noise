#pragma once

#include <stdint.h>

typedef struct pcg_state_setseq_64 { // Internals are *Private*.
  uint64_t state;                    // RNG state.  All values are possible.
  uint64_t inc;                      // Controls which RNG sequence (stream) is
                                     // selected. Must *always* be odd.
} pcg32_random_t;

static inline uint32_t pcg32_random_r(pcg32_random_t *rng) {
  uint64_t oldstate = rng->state;
  rng->state = oldstate * 6364136223846793005ULL + rng->inc;
  uint32_t xorshifted = (uint32_t)((oldstate >> 18u) ^ oldstate) >> 27u;
  uint32_t rot = oldstate >> 59u;
  return (xorshifted >> rot) | (xorshifted << ((uint32_t)(-rot) & 31));
}

static inline uint32_t java_random_bounded(uint32_t bound,
                                           pcg32_random_t *rng) {
  uint32_t rkey = pcg32_random_r(rng);
  uint32_t candidate = rkey % bound;
  while (rkey - candidate > UINT32_MAX - bound + 1) {
    rkey = pcg32_random_r(rng);
    candidate = rkey % bound;
  }
  return candidate;
}

inline void shuffle_pcg(int *arr, uint32_t size, uint64_t seed) {
  pcg_state_setseq_64 rng = {seed, 0xda3e39cb94b95bdbULL};
  uint32_t i, t;
  for (i = size; i > 1; i--) {
    t = java_random_bounded(i, &rng);
    int tmp = arr[i - 1];
    arr[i - 1] = arr[t];
    arr[t] = tmp;
  }
}
