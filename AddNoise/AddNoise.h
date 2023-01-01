#pragma once

#include <type_traits>
#include <vector>

#include <VSHelper4.h>
#include <VapourSynth4.h>

#include "pcg.h"
#include <algorithm>
#include <random>

#include "perlin.h"
#include "poisson.h"
#include "simplex.h"

#ifdef ADDGRAIN_X86
#include "VCL2/vectorclass.h"
#endif

// max # of noise planes
static constexpr int MAXP = 2;

// offset in pixels of the fake plane MAXP relative to plane MAXP-1
static constexpr int OFFSET_FAKEPLANE = 32;

struct AddNoiseData final {
  VSNode *node;
  const VSVideoInfo *vi;
  int type;

  // Gaussian
  float var;
  float uvar;
  float hcorr;
  float vcorr;

  // Perlin
  float xsize;
  float ysize;

  uint64_t idum;
  bool constant;
  int every;

  float scale;
  int planesNoise;
  bool process[3];

  int nStride[MAXP];
  int nHeight[MAXP];
  int nSize[MAXP];
  int peak;
  int step;
  int storedFrames;

  std::default_random_engine generator;

  std::vector<uint64_t> pNoiseSeeds;
  std::vector<std::vector<int8_t>> pN;
  void (*updateFrame)(const void *_srcp, void *_dstp, const int width,
                      const int height, const ptrdiff_t stride,
                      const int noisePlane, void *pNW,
                      const AddNoiseData *const VS_RESTRICT d) noexcept;
};
