# AddNoise
AddNoise generates film like noise or other effects (like rain) by adding random noise to a video clip. This noise may optionally be horizontally or vertically correlated to cause streaking.

Original https://github.com/HomeOfVapourSynthEvolution/VapourSynth-AddGrain ported from AviSynth plugin http://forum.doom9.org/showthread.php?t=111849

| | | | |
| - | - | - | - |
| ![type=0](https://user-images.githubusercontent.com/19401176/156282186-3327868b-2cae-41bc-8b10-e8304fae8695.png) | ![type=1](https://user-images.githubusercontent.com/19401176/156282191-0709415f-fdfa-46a6-aadf-db8be1c243bf.png) | ![type=2](https://user-images.githubusercontent.com/19401176/156282249-61716279-9926-43ad-a387-7e007f4e10f0.png) | ![type=3](https://user-images.githubusercontent.com/19401176/156282199-b60b26c1-df26-43e6-96e6-b44c4b4aea7a.png) |



## Usage
    noise.Add(vnode clip[, int type=0, float var=1.0, float uvar=0.0, float hcorr=0.0, float vcorr=0.0, float xsize = 2.0, float ysize = 2.0, int seed=-1, int constant=False, int every=1, int opt=0])

- clip: Clip to process. Any format with either integer sample type of 8-16 bit depth or float sample type of 32 bit depth is supported.

- type: Type of noise.
  - 0 = Gaussian
  - 1 = Perlin
  - 2 = Simplex
  - 3 = Fractional Brownian Motion over simplex

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
