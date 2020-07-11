/*
 * Copyright (C) 2005 Michael Niedermayer <michaelni@gmx.at>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>

#include "avisynth.h"
#include "avs/minmax.h"

static constexpr int N = 1 << 16;
static constexpr int N0 = 4;
static constexpr int N1 = 5;
static constexpr int N2 = 10;
static constexpr int SN0 = 2;
static constexpr double SN2 = 3.1622776601683795;

template<typename T, int scale>
static inline void dctA(const T* srcp, T* __restrict dstp, const int stride) noexcept
{
    for (int i = 0; i < 4; i++)
    {
        T s0 = (srcp[0 * stride] + srcp[6 * stride]) * scale;
        T s1 = (srcp[1 * stride] + srcp[5 * stride]) * scale;
        T s2 = (srcp[2 * stride] + srcp[4 * stride]) * scale;
        T s3 = srcp[3 * stride] * scale;
        T s = s3 + s3;
        s3 = s - s0;
        s0 = s + s0;
        s = s2 + s1;
        s2 = s2 - s1;
        dstp[0] = s0 + s;
        dstp[2] = s0 - s;
        dstp[1] = 2 * s3 + s2;
        dstp[3] = s3 - 2 * s2;

        srcp++;
        dstp += 4;
    }
}

template<typename T>
static inline void dctB(const T* srcp, T* __restrict dstp) noexcept
{
    for (int i = 0; i < 4; i++) {
        T s0 = srcp[0 * 4] + srcp[6 * 4];
        T s1 = srcp[1 * 4] + srcp[5 * 4];
        T s2 = srcp[2 * 4] + srcp[4 * 4];
        T s3 = srcp[3 * 4];
        T s = s3 + s3;
        s3 = s - s0;
        s0 = s + s0;
        s = s2 + s1;
        s2 = s2 - s1;
        dstp[0 * 4] = s0 + s;
        dstp[2 * 4] = s0 - s;
        dstp[1 * 4] = 2 * s3 + s2;
        dstp[3 * 4] = s3 - 2 * s2;

        srcp++;
        dstp++;
    }
}

class vsDeblockPP7 : public GenericVideoFilter
{
    int mode_, y_, u_, v_;
    int process[3];
    int stride[3];
    unsigned thresh[16], peak;
    int* buffer;
    bool has_at_least_v8;

    template<typename T>
    inline void pp7Filter_c(uint8_t* __restrict dstp_, const uint8_t* srcp_, int srcStride, int dst_strinde, const int stride, const int width, const int height) noexcept;

public:
    vsDeblockPP7(PClip _child, double qp, int mode, int y, int u, int v, IScriptEnvironment* env);
    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
    int __stdcall SetCacheHints(int cachehints, int frame_range)
    {
        return cachehints == CACHE_GET_MTMODE ? MT_MULTI_INSTANCE : 0;
    }
    ~vsDeblockPP7();

    const int16_t factor[16] = {
        N / (N0 * N0), N / (N0 * N1), N / (N0 * N0), N / (N0 * N2),
        N / (N1 * N0), N / (N1 * N1), N / (N1 * N0), N / (N1 * N2),
        N / (N0 * N0), N / (N0 * N1), N / (N0 * N0), N / (N0 * N2),
        N / (N2 * N0), N / (N2 * N1), N / (N2 * N0), N / (N2 * N2)
    };

};

template<typename T>
inline void vsDeblockPP7::pp7Filter_c(uint8_t* __restrict dstp_, const uint8_t* srcp_, int srcStride, int dst_strinde, const int stride, const int width, const int height) noexcept
{
    if constexpr (std::is_same_v<T, float>)
    {
        float* buffer = reinterpret_cast<float*>(vsDeblockPP7::buffer);

        srcStride /= sizeof(float);
        dst_strinde /= sizeof(float);

        const float* srcp = reinterpret_cast<const float*>(srcp_);
        float* __restrict dstp = reinterpret_cast<float* __restrict>(dstp_);

        float* __restrict p_src = buffer + stride * static_cast<int64_t>(8);
        float* __restrict block = buffer;
        float* __restrict temp = buffer + 16;

        for (int y = 0; y < height; y++)
        {
            const int index = stride * (8 + y) + 8;
            std::copy_n(srcp + srcStride * static_cast<int64_t>(y), width, p_src + index);

            for (int x = 0; x < 8; x++)
            {
                p_src[index - 1 - x] = p_src[index + x];
                p_src[index + width + x] = p_src[index + width - 1 - x];
            }
        }

        for (int y = 0; y < 8; y++) {
            memcpy(p_src + stride * (static_cast<int64_t>(7) - y), p_src + stride * (static_cast<int64_t>(8) + y), stride * sizeof(float));
            memcpy(p_src + stride * (height + static_cast<int64_t>(8) + y), p_src + stride * (height + static_cast<int64_t>(7) - y), stride * sizeof(float));
        }

        for (int y = 0; y < height; y++)
        {
            for (int x = -8; x < 0; x += 4)
            {
                const int index = (stride + 1) * (8 - 3) + stride * y + 8 + x;
                float* __restrict tp = temp + static_cast<int64_t>(4) * x;

                dctA<float, 255>(p_src + index, tp + 4 * 8, stride);
            }

            for (int x = 0; x < width; x++)
            {
                const int index = (stride + 1) * (8 - 3) + stride * y + 8 + x;
                float* __restrict tp = temp + static_cast<int64_t>(4) * x;

                if (!(x & 3))
                    dctA<float, 255>(p_src + index, tp + 4 * 8, stride);
                dctB(tp, block);

                float v = block[0] * factor[0];

                if (mode_ == 0)
                {
                    for (int i = 1; i < 16; i++)
                    {
                        const unsigned threshold1 = thresh[i];
                        const unsigned threshold2 = threshold1 * 2;

                        if (static_cast<unsigned>(block[i]) + threshold1 > threshold2)
                            v += block[i] * factor[i];
                    }
                }
                else if (mode_ == 1)
                {
                    for (int i = 1; i < 16; i++)
                    {
                        const unsigned threshold1 = thresh[i];
                        const unsigned threshold2 = threshold1 * 2;

                        if (static_cast<unsigned>(block[i]) + threshold1 > threshold2)
                        {
                            if (block[i] > 0.f)
                                v += (block[i] - threshold1) * factor[i];
                            else
                                v += (block[i] + threshold1) * factor[i];
                        }
                    }
                }
                else
                {
                    for (int i = 1; i < 16; i++)
                    {
                        const unsigned threshold1 = thresh[i];
                        const unsigned threshold2 = threshold1 * 2;

                        if (static_cast<unsigned>(block[i]) + threshold1 > threshold2)
                        {
                            if (static_cast<unsigned>(block[i]) + threshold2 > threshold2 * 2)
                            {
                                v += block[i] * factor[i];
                            }
                            else
                            {
                                if (block[i] > 0.f)
                                    v += 2.f * (block[i] - threshold1) * factor[i];
                                else
                                    v += 2.f * (block[i] + threshold1) * factor[i];
                            }
                        }
                    }
                }

                dstp[dst_strinde * y + x] = v * ((1.f / (1 << 18)) * (1.f / 255.f));
            }
        }
    }
    else
    {
        int* buffer = vsDeblockPP7::buffer;

        srcStride /= sizeof(T);
        dst_strinde /= sizeof(T);

        const T* srcp = reinterpret_cast<const T*>(srcp_);
        T* __restrict dstp = reinterpret_cast<T * __restrict>(dstp_);

        int* __restrict p_src = buffer + stride * static_cast<int64_t>(8);
        int* __restrict block = buffer;
        int* __restrict temp = buffer + 16;

        for (int y = 0; y < height; y++)
        {
            const int index = stride * (8 + y) + 8;
            std::copy_n(srcp + srcStride * static_cast<int64_t>(y), width, p_src + index);
            for (int x = 0; x < 8; x++)
            {
                p_src[index - 1 - x] = p_src[index + x];
                p_src[index + width + x] = p_src[index + width - 1 - x];
            }
        }

        for (int y = 0; y < 8; y++)
        {
            memcpy(p_src + stride * (static_cast<int64_t>(7) - y), p_src + stride * (static_cast<int64_t>(8) + y), stride * sizeof(int));
            memcpy(p_src + stride * (height + static_cast<int64_t>(8) + y), p_src + stride * (height + static_cast<int64_t>(7) - y), stride * sizeof(int));
        }

        for (int y = 0; y < height; y++)
        {
            for (int x = -8; x < 0; x += 4)
            {
                const int index = (stride + 1) * (8 - 3) + stride * y + 8 + x;
                int* __restrict tp = temp + static_cast<int64_t>(4) * x;

                dctA<int, 1>(p_src + index, tp + 4 * 8, stride);
            }

            for (int x = 0; x < width; x++) {
                const int index = (stride + 1) * (8 - 3) + stride * y + 8 + x;
                int* __restrict tp = temp + static_cast<int64_t>(4) * x;

                if (!(x & 3))
                    dctA<int, 1>(p_src + index, tp + 4 * 8, stride);
                dctB(tp, block);

                int64_t v = static_cast<int64_t>(block[0]) * factor[0];

                if (mode_ == 0) {
                    for (int i = 1; i < 16; i++)
                    {
                        const unsigned threshold1 = thresh[i];
                        const unsigned threshold2 = threshold1 * 2;
                        if (block[i] + threshold1 > threshold2)
                            v += static_cast<int64_t>(block[i]) * factor[i];
                    }
                }
                else if (mode_ == 1)
                {
                    for (int i = 1; i < 16; i++)
                    {
                        const unsigned threshold1 = thresh[i];
                        const unsigned threshold2 = threshold1 * 2;
                        if (block[i] + threshold1 > threshold2)
                        {
                            if (block[i] > 0)
                                v += (block[i] - static_cast<int64_t>(threshold1)) * factor[i];
                            else
                                v += (block[i] + static_cast<int64_t>(threshold1)) * factor[i];
                        }
                    }
                }
                else
                {
                    for (int i = 1; i < 16; i++)
                    {
                        const unsigned threshold1 = thresh[i];
                        const unsigned threshold2 = threshold1 * 2;
                        if (block[i] + threshold1 > threshold2)
                        {
                            if (block[i] + threshold2 > threshold2 * 2) {
                                v += static_cast<int64_t>(block[i]) * factor[i];
                            }
                            else
                            {
                                if (block[i] > 0)
                                    v += 2 * (block[i] - static_cast<int64_t>(threshold1)) * factor[i];
                                else
                                    v += 2 * (block[i] + static_cast<int64_t>(threshold1)) * factor[i];
                            }
                        }
                    }
                }

                v = (v + (1 << 17)) >> 18;

                if (static_cast<unsigned>(v) > peak)
                    v = -v >> 63;

                dstp[dst_strinde * y + x] = static_cast<T>(v);
            }
        }
    }
}

vsDeblockPP7::vsDeblockPP7(PClip _child, double qp, int mode, int y, int u, int v, IScriptEnvironment* env)
    : GenericVideoFilter(_child), mode_(mode), y_(y), u_(u), v_(v)
{
    if (!vi.IsPlanar())
        env->ThrowError("vsDeblockPP7: only constant format 8-16 bit integer and 32 bit float input supported");

    if (qp < 1. || qp > 63.)
        env->ThrowError("vsDeblockPP7: qp must be between 1.0 and 63.0 (inclusive)");

    if (mode_ < 0 || mode_ > 2)
        env->ThrowError("vsDeblockPP7: mode must be 0, 1 or 2");

    peak = !(vi.ComponentSize() == 4) ? (1 << vi.BitsPerComponent()) - 1 : 255;

    for (int plane = 0; plane < vi.NumComponents(); plane++)
    {
        const int width = vi.width >> ((plane && !vi.IsRGB())? vi.GetPlaneWidthSubsampling(PLANAR_U) : 0);
        stride[plane] = (width + 16 + 15) & ~15;
    }

    for (int i = 0; i < 16; i++)
        thresh[i] = static_cast<unsigned>((((i & 1) ? SN2 : SN0) * ((i & 4) ? SN2 : SN0) * qp * (static_cast<int64_t>(1) << 2) - 1) * peak / 255.);

    int planecount = min(vi.NumComponents(), 3);
    for (int i = 0; i < planecount; i++)
    {
        if (vi.IsRGB())
            process[i] = 3;
        else
        {
            switch (i)
            {
            case 0:
                switch (y_)
                {
                case 3:
                    process[i] = 3;
                    break;
                case 2:
                    process[i] = 2;
                    break;
                default:
                    process[i] = 1;
                    break;
                }
                break;
            case 1:
                switch (u_)
                {
                case 3:
                    process[i] = 3;
                    break;
                case 2:
                    process[i] = 2;
                    break;
                default:
                    process[i] = 1;
                    break;
                }
                break;
            default:
                switch (v_)
                {
                case 3:
                    process[i] = 3;
                    break;
                case 2:
                    process[i] = 2;
                    break;
                default:
                    process[i] = 1;
                    break;
                }
                break;
            }
        }
    }

    has_at_least_v8 = true;
    try { env->CheckVersion(8); }
    catch (const AvisynthError&) { has_at_least_v8 = false; }

    buffer = reinterpret_cast<int*>(_aligned_malloc(stride[0] * (vi.height + static_cast<int64_t>(16) + 8) * sizeof(int), 16));
}

vsDeblockPP7::~vsDeblockPP7()
{
    _aligned_free(buffer);
}

static inline void copy_plane(PVideoFrame& dst, PVideoFrame& src, int plane, IScriptEnvironment* env)
{
    const uint8_t* srcp = src->GetReadPtr(plane);
    int src_pitch = src->GetPitch(plane);
    int height = src->GetHeight(plane);
    int row_size = src->GetRowSize(plane);
    uint8_t* destp = dst->GetWritePtr(plane);
    int dst_pitch = dst->GetPitch(plane);
    env->BitBlt(destp, dst_pitch, srcp, src_pitch, row_size, height);
}

PVideoFrame __stdcall vsDeblockPP7::GetFrame(int n, IScriptEnvironment* env)
{
    PVideoFrame src = child->GetFrame(n, env);
    PVideoFrame dst = has_at_least_v8 ? env->NewVideoFrameP(vi, &src, 16) : env->NewVideoFrame(vi, 16);

    int planes_y[4] = { PLANAR_Y, PLANAR_U, PLANAR_V, PLANAR_A };
    int planes_r[4] = { PLANAR_G, PLANAR_B, PLANAR_R, PLANAR_A };
    const int* current_planes = (vi.IsYUV() || vi.IsYUVA()) ? planes_y : planes_r;
    const int planecount = std::min(vi.NumComponents(), 3);
    for (int i = 0; i < planecount; i++)
    {
        const int plane = current_planes[i];

        int srcStride = src->GetPitch(plane);
        int dst_stride = dst->GetPitch(plane);
        const int stride = vsDeblockPP7::stride[i];
        const int comp = vi.ComponentSize();
        const int width = src->GetRowSize(plane) / comp;
        const int height = src->GetHeight(plane);
        const uint8_t* srcp = src->GetReadPtr(plane);
        uint8_t* __restrict dstp = dst->GetWritePtr(plane);

        switch (process[i])
        {
        case 3:
            switch (comp)
            {
            case 1:
                pp7Filter_c<uint8_t>(dstp, srcp, srcStride, dst_stride, stride, width, height);
                break;
            case 2:
                pp7Filter_c<uint16_t>(dstp, srcp, srcStride, dst_stride, stride, width, height);
                break;
            default:
                pp7Filter_c<float>(dstp, srcp, srcStride, dst_stride, stride, width, height);
                break;
            }
            break;
        case 2:
            copy_plane(dst, src, plane, env);
            break;
        }
    }

    return dst;
}

AVSValue __cdecl Create_vsDeblockPP7(AVSValue args, void* user_data, IScriptEnvironment* env)
{
    return new vsDeblockPP7(
        args[0].AsClip(),
        args[1].AsFloat(2.0),
        args[2].AsInt(0),
        args[3].AsInt(3),
        args[4].AsInt(3),
        args[5].AsInt(3),
        env);
}

const AVS_Linkage* AVS_linkage;

extern "C" __declspec(dllexport)
const char* __stdcall AvisynthPluginInit3(IScriptEnvironment * env, const AVS_Linkage* const vectors)
{
    AVS_linkage = vectors;

    env->AddFunction("vsDeblockPP7", "c[qp]f[mode]i[y]i[u]i[v]i", Create_vsDeblockPP7, 0);

    return "vsDeblockPP7";
}
