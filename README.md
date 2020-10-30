# Description

Variant of the spp filter in MPlayer, similar to spp=6 with 7 point DCT where only the center sample is used after IDCT.

This is [a port of the VapourSynth plugin DeblockPP7](https://github.com/HomeOfVapourSynthEvolution/VapourSynth-DeblockPP7).

# Usage

```
vsDeblockPP7 (clip, float "qp", int "mode", int "y", int "u", int "v")
```

## Parameters:

- clip\
    A clip to process. It must be in planar format.
    
- qp\
    Constant quantization parameter.\
    Must be between 1.0 and 63.0.\
    Default: 2.0.
    
- mode\
    0: Hard threshold.\
    1: Soft threshold (better deringing, but blurrier).\
    2: Medium threshold (compromise between hard and soft).\
    Default: 0.
    
- y, u, v\
    Planes to process.\
    1: Return garbage.\
    2: Copy plane.\
    3: Process plane. Always process planes when the clip is RGB.\
    Default: y = 3, u = 3, v = 3.
    