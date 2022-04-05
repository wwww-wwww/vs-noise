// Copyright (c) 2002 Tom Barry.  All rights reserved.
//      trbarry@trbarry.com
//  modified by Foxyshadis
//      foxyshadis@hotmail.com
//  modified by Firesledge
//      http://ldesoras.free.fr
//  modified by LaTo INV.
//      http://forum.doom9.org/member.php?u=131032
//  VapourSynth port by HolyWu
//      https://github.com/HomeOfVapourSynthEvolution/VapourSynth-AddNoise
//  Noise by w
//      https://github.com/wwww-wwww/vs-noise
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <cmath>
#include <ctime>

#include <algorithm>
#include <memory>
#include <string>

#include "AddNoise.h"

using namespace std::literals;

#ifdef ADDGRAIN_X86
template <typename pixel_t, typename noise_t>
extern void updateFrame_sse2(const void *_srcp, void *_dstp, const int width,
                             const int height, const ptrdiff_t stride,
                             const int noisePlane, void *_pNW,
                             const AddNoiseData *const VS_RESTRICT d) noexcept;
template <typename pixel_t, typename noise_t>
extern void updateFrame_avx2(const void *_srcp, void *_dstp, const int width,
                             const int height, const ptrdiff_t stride,
                             const int noisePlane, void *_pNW,
                             const AddNoiseData *const VS_RESTRICT d) noexcept;
template <typename pixel_t, typename noise_t>
extern void
updateFrame_avx512(const void *_srcp, void *_dstp, const int width,
                   const int height, const ptrdiff_t stride,
                   const int noisePlane, void *_pNW,
                   const AddNoiseData *const VS_RESTRICT d) noexcept;
#endif

template <typename T>
static T getArg(const VSAPI *vsapi, const VSMap *map, const char *key,
                const T defaultValue) noexcept {
  T arg{};
  int err{0};

  if constexpr (std::is_same_v<T, bool>)
    arg = !!vsapi->mapGetInt(map, key, 0, &err);
  else if constexpr (std::is_same_v<T, int> || std::is_same_v<T, long>)
    arg = vsapi->mapGetIntSaturated(map, key, 0, &err);
  else if constexpr (std::is_same_v<T, int64_t>)
    arg = vsapi->mapGetInt(map, key, 0, &err);
  else if constexpr (std::is_same_v<T, float>)
    arg = vsapi->mapGetFloatSaturated(map, key, 0, &err);
  else if constexpr (std::is_same_v<T, double>)
    arg = vsapi->mapGetFloat(map, key, 0, &err);

  if (err)
    arg = defaultValue;

  return arg;
}

uint64_t LCG(uint64_t &idum) noexcept {
  return idum = 1664525L * idum + 1013904223L;
}

// very fast & reasonably random
static inline float fastUniformRandF(uint64_t &idum) noexcept {
  // work with 32-bit IEEE floating point only!
  LCG(idum);
  unsigned long itemp = 0x3f800000 | (0x007fffff & idum);
  return *reinterpret_cast<float *>(&itemp) - 1.0f;
}

static inline float gaussianRand(bool &iset, float &gset,
                                 uint64_t &idum) noexcept {
  float fac, rsq, v1, v2;

  // return saved second
  if (iset) {
    iset = false;
    return gset;
  }

  do {
    v1 = 2.0f * fastUniformRandF(idum) - 1.0f;
    v2 = 2.0f * fastUniformRandF(idum) - 1.0f;
    rsq = v1 * v1 + v2 * v2;
  } while (rsq >= 1.0f || rsq == 0.0f);

  fac = std::sqrt(-2.0f * std::log(rsq) / rsq);

  // function generates two values every iteration, so save one for later
  gset = v1 * fac;
  iset = true;

  return v2 * fac;
}

static inline float gaussianRand(const float mean, const float variance,
                                 bool &iset, float &gset,
                                 uint64_t &idum) noexcept {
  return (variance == 0.0f)
             ? mean
             : gaussianRand(iset, gset, idum) * std::sqrt(variance) + mean;
}

template <typename noise_t>
static void generateNoise(AddNoiseData *const VS_RESTRICT d) noexcept {
  float nRep[MAXP];
  for (auto i{0}; i < MAXP; i++) {
    nRep[i] = d->constant ? 1.0f : 2.0f;
  }

  std::vector<float> lastLine(d->nStride[0]); // assume plane 0 is the widest
  constexpr auto mean{0.0f};
  const float pvar[]{d->var, d->uvar};
  bool iset{false};
  float gset{0.0f};

  for (auto plane{0}; plane < d->planesNoise; plane++) {
    auto h{static_cast<int>(std::ceil(d->nHeight[plane] * nRep[plane]))};
    if (d->planesNoise == 2 && plane == 1) {
      // fake plane needs at least one more row, and more if the rows are too
      // small. round to the upper number
      h += (OFFSET_FAKEPLANE + d->nStride[plane] - 1) / d->nStride[plane];
    }
    d->nSize[plane] = d->nStride[plane] * h;

    // allocate space for noise
    d->pN[plane].resize(d->nSize[plane] * sizeof(noise_t));

    for (auto x{0}; x < d->nStride[plane]; x++) {
      // things to vertically smooth against
      if (d->type == 0) {
        lastLine[x] = gaussianRand(mean, pvar[plane], iset, gset, d->idum);
      }
    }

    for (auto y{0}; y < h; y++) {
      auto pNW{reinterpret_cast<noise_t *>(d->pN[plane].data()) +
               d->nStride[plane] * y};
      auto lastr{gaussianRand(mean, pvar[plane], iset, gset,
                              d->idum)}; // something to horiz smooth against

      for (auto x{0}; x < d->nStride[plane]; x++) {
        auto r{gaussianRand(mean, pvar[plane], iset, gset, d->idum)};

        r = lastr * d->hcorr + r * (1.0f - d->hcorr); // horizontal corr
        lastr = r;

        r = lastLine[x] * d->vcorr + r * (1.0f - d->vcorr); // vert corr
        lastLine[x] = r;

        // set noise block
        if constexpr (std::is_integral_v<noise_t>) {
          *pNW++ = static_cast<noise_t>(std::round(r) * d->scale);
        } else {
          *pNW++ = r * d->scale;
        }
      }
    }
  }
}

static void generate_seeds(AddNoiseData *const VS_RESTRICT d) noexcept {
  auto pns{d->pNoiseSeeds.begin()};

  for (auto plane{0}; plane < d->planesNoise; plane++) {
    for (auto x{d->storedFrames}; x > 0; x--) {
      *pns++ = LCG(d->idum); // insert seed, to keep cache happy
    }
  }
}

// on input, plane is the frame plane index (if applicable, 0 otherwise), and on
// output, it contains the selected noise plane
static void setRand(int &plane, int &noiseOffs, const int frameNumber,
                    AddNoiseData *const VS_RESTRICT d) noexcept {
  if (d->constant) {
    // force noise to be identical every frame
    if (plane >= MAXP) {
      plane = MAXP - 1;
      noiseOffs = OFFSET_FAKEPLANE;
    }
  } else {
    // pull seed back out, to keep cache happy
    uint8_t seedIndex{
        static_cast<uint8_t>((frameNumber / d->every) % d->storedFrames)};
    uint64_t p0{d->pNoiseSeeds[seedIndex]};

    if (plane == 0) {
      d->idum = p0;
    } else {
      d->idum = d->pNoiseSeeds[seedIndex + d->storedFrames];
      if (plane == 2) {
        // the trick to needing only 2 planes ^.~
        d->idum ^= p0;
        plane--;
      }
    }

    // start noise at random qword in top half of noise area
    noiseOffs =
        static_cast<int>(fastUniformRandF(d->idum) * d->nSize[plane] / MAXP) &
        0xfffffff8;
  }
}

template <typename pixel_t, typename noise_t>
static void updateFrame_c(const void *_srcp, void *_dstp, const int width,
                          const int height, const ptrdiff_t stride,
                          const int noisePlane, void *_pNW,
                          const AddNoiseData *const VS_RESTRICT d) noexcept {
  auto srcp{reinterpret_cast<const pixel_t *>(_srcp)};
  auto dstp{reinterpret_cast<pixel_t *>(_dstp)};
  auto pNW{reinterpret_cast<noise_t *>(_pNW)};

  for (auto y{0}; y < height; y++) {
    for (auto x{0}; x < width; x++) {
      if constexpr (std::is_integral_v<pixel_t>) {
        dstp[x] =
            static_cast<pixel_t>(std::clamp(srcp[x] + pNW[x], 0, d->peak));
      } else {
        dstp[x] = srcp[x] + pNW[x];
      }
    }

    srcp += stride;
    dstp += stride;
    pNW += d->nStride[noisePlane];
  }
}

template <typename pix_fmt>
static void create_noise(int n, std::vector<std::vector<int8_t>> &planes,
                         AddNoiseData *const VS_RESTRICT d) {
  const float pvar[]{d->var, d->uvar};

  planes.resize(d->vi->format.numPlanes);

  int p[PERM_SIZE];
  for (int x = 0; x < PERM_SIZE; x++) {
    p[x] = x;
  }

  uint8_t seedIndex = 0;
  if (!d->constant) {
    seedIndex = static_cast<uint8_t>((n / d->every) % d->storedFrames);
  }
  uint64_t p0 = d->pNoiseSeeds[seedIndex];

  for (int i = 0; i < d->vi->format.numPlanes; i++) {
    if (d->constant) {
      LCG(d->idum);
      shuffle_pcg(p, PERM_SIZE, d->idum);
    } else {
      shuffle_pcg(p, PERM_SIZE,
                  i == 0 ? p0 : d->pNoiseSeeds[seedIndex + d->storedFrames]);
    }

    auto noisePlane{d->vi->format.colorFamily == cfRGB ? 0 : i};
    if (noisePlane >= 2) {
      noisePlane = 1;
    }

    planes[i].resize(d->nStride[noisePlane] * d->nHeight[noisePlane] *
                     sizeof(pix_fmt));

    if (d->type == 1) {
      perlin_create<pix_fmt>((void *)planes[i].data(), d->nStride[noisePlane],
                             d->nHeight[noisePlane], d->xsize, d->ysize,
                             pvar[noisePlane] * d->scale, p);
    } else if (d->type == 2) {
      simplex_create<pix_fmt>((void *)planes[i].data(), d->nStride[noisePlane],
                              d->nHeight[noisePlane], d->xsize, d->ysize,
                              pvar[noisePlane] * d->scale, p);
    } else if (d->type == 3) {
      fractal_create<pix_fmt>((void *)planes[i].data(), d->nStride[noisePlane],
                              d->nHeight[noisePlane], d->xsize, d->ysize,
                              pvar[noisePlane] * d->scale, p, 5, 1.0, 1.0, 2.0,
                              0.5);
    }
  }
}

static const VSFrame *VS_CC addnoiseGetFrame(int n, int activationReason,
                                             void *instanceData,
                                             [[maybe_unused]] void **frameData,
                                             VSFrameContext *frameCtx,
                                             VSCore *core, const VSAPI *vsapi) {
  auto d{static_cast<AddNoiseData *>(instanceData)};

  if (activationReason == arInitial) {
    vsapi->requestFrameFilter(n, d->node, frameCtx);
  } else if (activationReason == arAllFramesReady) {
    auto src{vsapi->getFrameFilter(n, d->node, frameCtx)};
    decltype(src) fr[]{d->process[0] ? nullptr : src,
                       d->process[1] ? nullptr : src,
                       d->process[2] ? nullptr : src};
    constexpr int pl[]{0, 1, 2};
    auto dst{vsapi->newVideoFrame2(&d->vi->format, d->vi->width, d->vi->height,
                                   fr, pl, src, core)};

    std::vector<std::vector<int8_t>> planes;
    std::vector<std::vector<int8_t>> *planes_ptr;

    if (d->constant || d->type == 0) {
      planes_ptr = &d->pN;
    } else {
      if (d->vi->format.bytesPerSample == 1) {
        create_noise<int8_t>(n, planes, d);
      } else if (d->vi->format.bytesPerSample == 2) {
        create_noise<int16_t>(n, planes, d);
      } else {
        create_noise<float>(n, planes, d);
      }
      planes_ptr = &planes;
    }

    for (auto plane{0}; plane < d->vi->format.numPlanes; plane++) {
      if (d->process[plane]) {
        const auto width{vsapi->getFrameWidth(src, plane)};
        const auto height{vsapi->getFrameHeight(src, plane)};
        const auto stride{vsapi->getStride(src, plane) /
                          d->vi->format.bytesPerSample};
        auto srcp{vsapi->getReadPtr(src, plane)};
        auto dstp{vsapi->getWritePtr(dst, plane)};

        auto noisePlane{d->vi->format.colorFamily == cfRGB ? 0 : plane};
        auto noiseOffs{0};

        switch (d->type) {
        case 0: { // Gaussian
          setRand(noisePlane, noiseOffs, n, d);
          void *pNW{((int8_t *)planes_ptr->at(noisePlane).data()) + noiseOffs};
          d->updateFrame(srcp, dstp, width, height, stride, noisePlane, pNW, d);

          break;
        }
        case 1:   // Perlin
        case 2:   // Simplex
        case 3: { // Fractal
          if (noisePlane >= 2) {
            noisePlane = 1;
          }
          void *pNW = planes_ptr->at(plane).data();
          d->updateFrame(srcp, dstp, width, height, stride, noisePlane, pNW, d);
          break;
        }
        default:
          break;
        }
      }
    }

    vsapi->freeFrame(src);
    return dst;
  }

  return nullptr;
}

static void VS_CC addnoiseFree(void *instanceData,
                               [[maybe_unused]] VSCore *core,
                               const VSAPI *vsapi) {
  auto d{static_cast<AddNoiseData *>(instanceData)};

  vsapi->freeNode(d->node);

  delete d;
}

static void VS_CC addnoise_create(const VSMap *in, VSMap *out,
                                  [[maybe_unused]] void *userData, VSCore *core,
                                  const VSAPI *vsapi) {
  auto d{std::make_unique<AddNoiseData>()};

  try {
    d->node = vsapi->mapGetNode(in, "clip", 0, nullptr);
    d->vi = vsapi->getVideoInfo(d->node);

    if (!vsh::isConstantVideoFormat(d->vi) ||
        (d->vi->format.sampleType == stInteger &&
         d->vi->format.bitsPerSample > 16) ||
        (d->vi->format.sampleType == stFloat &&
         d->vi->format.bitsPerSample != 32)) {
      throw "only constant format 8-16 bit integer and 32 bit float input "
            "supported";
    }

    d->type = getArg(vsapi, in, "type", 0);

    // Gaussian
    d->var = getArg(vsapi, in, "var", 1.0f);
    d->uvar = getArg(vsapi, in, "uvar", 0.0f);
    d->hcorr = getArg(vsapi, in, "hcorr", 0.0f);
    d->vcorr = getArg(vsapi, in, "vcorr", 0.0f);

    // Perlin
    d->xsize = getArg(vsapi, in, "xsize", 2.0f);
    d->ysize = getArg(vsapi, in, "ysize", 2.0f);

    d->idum = getArg(vsapi, in, "seed", -1L);
    d->constant = getArg(vsapi, in, "constant", false);

    d->every = getArg(vsapi, in, "every", 1);

    auto opt = getArg(vsapi, in, "opt", 0);

    if (d->hcorr < 0.0f || d->hcorr > 1.0f || d->vcorr < 0.0f ||
        d->vcorr > 1.0f) {
      throw "hcorr and vcorr must be between 0.0 and 1.0 (inclusive)";
    }

    if (opt < 0 || opt > 4) {
      throw "opt must be 0, 1, 2, 3, or 4";
    }

    if (d->type < 0 || d->type > 3) {
      throw "type must be 0, 1, 2, or 3";
    }

    if (d->vi->format.bytesPerSample == 1) {
      d->updateFrame = updateFrame_c<uint8_t, int8_t>;
    } else if (d->vi->format.bytesPerSample == 2) {
      d->updateFrame = updateFrame_c<uint16_t, int16_t>;
    } else {
      d->updateFrame = updateFrame_c<float, float>;
    }

#ifdef ADDGRAIN_X86
    auto iset{instrset_detect()};
    if ((opt == 0 && iset >= 10) || opt == 4) {
      if (d->vi->format.bytesPerSample == 1) {
        d->updateFrame = updateFrame_avx512<uint8_t, int8_t>;
        d->step = 64;
      } else if (d->vi->format.bytesPerSample == 2) {
        d->updateFrame = updateFrame_avx512<uint16_t, int16_t>;
        d->step = 32;
      } else {
        d->updateFrame = updateFrame_avx512<float, float>;
        d->step = 16;
      }
    } else if ((opt == 0 && iset >= 8) || opt == 3) {
      if (d->vi->format.bytesPerSample == 1) {
        d->updateFrame = updateFrame_avx2<uint8_t, int8_t>;
        d->step = 32;
      } else if (d->vi->format.bytesPerSample == 2) {
        d->updateFrame = updateFrame_avx2<uint16_t, int16_t>;
        d->step = 16;
      } else {
        d->updateFrame = updateFrame_avx2<float, float>;
        d->step = 8;
      }
    } else if ((opt == 0 && iset >= 2) || opt == 2) {
      if (d->vi->format.bytesPerSample == 1) {
        d->updateFrame = updateFrame_sse2<uint8_t, int8_t>;
        d->step = 16;
      } else if (d->vi->format.bytesPerSample == 2) {
        d->updateFrame = updateFrame_sse2<uint16_t, int16_t>;
        d->step = 8;
      } else {
        d->updateFrame = updateFrame_sse2<float, float>;
        d->step = 4;
      }
    }
#endif

    d->scale = 0.0f;
    if (d->vi->format.sampleType == stInteger) {
      d->scale = static_cast<float>(1 << (d->vi->format.bitsPerSample - 8));
      d->peak = (1 << d->vi->format.bitsPerSample) - 1;
    } else {
      d->scale = 1.0f / (d->vi->format.colorFamily == cfRGB ? 255.0f : 219.0f);
    }

    if (d->idum < 0) {
      d->idum = static_cast<uint64_t>(std::time(nullptr)); // init random
    }

    d->planesNoise = 1;
    d->nStride[0] = (d->vi->width + 63) & ~63;
    d->nHeight[0] = d->vi->height;
    if (d->vi->format.colorFamily == cfGray) {
      d->uvar = 0.0f;
    } else if (d->vi->format.colorFamily == cfRGB) {
      d->uvar = d->var;
    } else {
      d->planesNoise = 2;
      d->nStride[1] = ((d->vi->width >> d->vi->format.subSamplingW) + 63) & ~63;
      d->nHeight[1] = d->vi->height >> d->vi->format.subSamplingH;
    }

    if (d->var <= 0.0f && d->uvar <= 0.0f) {
      vsapi->mapConsumeNode(out, "clip", d->node, maReplace);
      return;
    }

    d->process[0] = d->var > 0.0f;
    d->process[1] = d->process[2] = d->uvar > 0.0f;

    d->storedFrames = std::min(d->vi->numFrames, 256);
    d->pNoiseSeeds.resize(d->storedFrames * d->planesNoise);

    generate_seeds(d.get());

    if (d->type == 0) {
      d->pN.resize(d->planesNoise);
      if (d->vi->format.bytesPerSample == 1) {
        generateNoise<int8_t>(d.get());
      } else if (d->vi->format.bytesPerSample == 2) {
        generateNoise<int16_t>(d.get());
      } else {
        generateNoise<float>(d.get());
      }
    } else {
      if (d->constant) {
        d->pN.resize(d->vi->format.numPlanes);
        if (d->vi->format.bytesPerSample == 1) {
          create_noise<int8_t>(0, d->pN, d.get());
        } else if (d->vi->format.bytesPerSample == 2) {
          create_noise<int16_t>(0, d->pN, d.get());
        } else {
          create_noise<float>(0, d->pN, d.get());
        }
      }
    }
  } catch (const char *error) {
    vsapi->mapSetError(out, ("AddNoise: "s + error).c_str());
    vsapi->freeNode(d->node);
    return;
  }

  VSFilterDependency deps[]{{d->node, rpStrictSpatial}};
  vsapi->createVideoFilter(out, "AddNoise", d->vi, addnoiseGetFrame,
                           addnoiseFree, fmParallel, deps, 1, d.get(), core);
  d.release();
}

//////////////////////////////////////////
// Init

VS_EXTERNAL_API(void)
VapourSynthPluginInit2(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
  vspapi->configPlugin("moe.grass.addnoise", "noise", "Noise generator",
                       VS_MAKE_VERSION(1, 0), VAPOURSYNTH_API_VERSION, 0,
                       plugin);
  vspapi->registerFunction("Add",
                           "clip:vnode;"
                           "var:float:opt;"
                           "uvar:float:opt;"
                           "type:int:opt;"
                           "hcorr:float:opt;"
                           "vcorr:float:opt;"
                           "xsize:float:opt;"
                           "ysize:float:opt;"
                           "scale:float:opt;"
                           "seed:int:opt;"
                           "constant:int:opt;"
                           "every:int:opt;"
                           "opt:int:opt;",
                           "clip:vnode;", addnoise_create, nullptr, plugin);
}
