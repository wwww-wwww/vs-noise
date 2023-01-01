# AddNoise

AddNoise generates film like noise or other effects (like rain) by adding random noise to a video clip. This noise may optionally be horizontally or vertically correlated to cause streaking.

Original https://github.com/HomeOfVapourSynthEvolution/VapourSynth-AddGrain ported from AviSynth plugin http://forum.doom9.org/showthread.php?t=111849

| | | | | | |
| - | - | - | - | - | - |
| ![ref](https://user-images.githubusercontent.com/19401176/210160826-878052da-0dc8-43b5-a7f1-53e597adda15.png) | ![type=0](https://user-images.githubusercontent.com/19401176/210161449-b5fadbf4-3537-4328-8068-fc3502cbea85.png) | ![type=1](https://user-images.githubusercontent.com/19401176/210161447-fe07ec8d-f114-4a38-a33b-0a8996fddc05.png) | ![type=2](https://user-images.githubusercontent.com/19401176/210161446-caba39c8-e15b-42c1-97f5-4f842712c88e.png) | ![type=3](https://user-images.githubusercontent.com/19401176/210161445-bd6969cf-417d-4672-afaa-9f0cb20df7e2.png) | ![type=4](https://user-images.githubusercontent.com/19401176/210161444-74b52123-cc7e-4043-99c2-fc582899a5ca.png)


## Usage
    noise.Add(vnode clip[, int type=0, float var=1.0, float uvar=0.0, float hcorr=0.0, float vcorr=0.0, float xsize = 2.0, float ysize = 2.0, int seed=-1, int constant=False, int every=1, int opt=0])

- clip: Clip to process. Any format with either integer sample type of 8-16 bit depth or float sample type of 32 bit depth is supported.

- type: Type of noise.

  - 0 = Gaussian
  - 1 = Perlin
  - 2 = Simplex
  - 3 = Fractional Brownian Motion over simplex
  - 4 = Poisson
    - Use var to control "sensitivity"
    - Should be used for intensity (Y for YUV) or RGB

- var, uvar: The variance (strength) of the luma and chroma noise, 0 is disabled. `uvar` does nothing for GRAY and RGB formats.

- hcorr, vcorr: Horizontal and vertical correlation, which causes a nifty streaking effect. Only for Gaussian noise. Range 0.0-1.0.

- xsize, ysize: size of Perlin and Simplex noise. Less than 2 for Perlin may generate visible boundaries.

- seed: Specifies a repeatable noise sequence. Set to at least 0 to use.

- constant: Specifies a constant noise pattern on every frame.

- every: Specify the period of frames for each noise pattern.

- opt: Sets which cpu optimizations to use.
  - 0 = auto detect
  - 1 = use c
  - 2 = use sse2
  - 3 = use avx2
  - 4 = use avx512

The correlation factors are actually just implemented as exponential smoothing which give a weird side affect that I did not attempt to adjust. But this means that as you increase either corr factor you will have to also increase the stddev (noise amount) in order to get the same visible amount of noise, since it is being smooth out a bit.

Increase both corr factors can somewhat give clumps, or larger noise size.

And there is an interesting effect with, say, `noise.Add(var=800, vcorr=0.9)` or any huge amount of strongly vertical noise. It can make a scene look like it is raining.

## Compilation

```
meson build
ninja -C build
ninja -C build install
```
