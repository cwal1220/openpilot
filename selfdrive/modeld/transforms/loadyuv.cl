#define UV_SIZE ((TRANSFORMED_WIDTH/2)*(TRANSFORMED_HEIGHT/2))

#define INTER_BITS 5
#define INTER_TAB_SIZE (1 << INTER_BITS)
#define INTER_REMAP_COEF_BITS 15
#define INTER_REMAP_COEF_SCALE (1 << INTER_REMAP_COEF_BITS)

__kernel void loadys(__global uchar8 const * const Y,
                     __global float * out,
                     int out_offset)
{
    const int gid = get_global_id(0);
    const int ois = gid * 8;
    const int oy = ois / TRANSFORMED_WIDTH;
    const int ox = ois % TRANSFORMED_WIDTH;

    const uchar8 ys = Y[gid];
    const float8 ysf = convert_float8(ys);

    // 02
    // 13

    __global float* outy0;
    __global float* outy1;
    if ((oy & 1) == 0) {
      outy0 = out + out_offset; //y0
      outy1 = out + out_offset + UV_SIZE*2; //y2
    } else {
      outy0 = out + out_offset + UV_SIZE; //y1
      outy1 = out + out_offset + UV_SIZE*3; //y3
    }

    vstore4(ysf.s0246, 0, outy0 + (oy/2) * (TRANSFORMED_WIDTH/2) + ox/2);
    vstore4(ysf.s1357, 0, outy1 + (oy/2) * (TRANSFORMED_WIDTH/2) + ox/2);
}

__kernel void loaduv(__global uchar8 const * const in,
                     __global float8 * out,
                     int out_offset)
{
  const int gid = get_global_id(0);
  const uchar8 inv = in[gid];
  const float8 outv  = convert_float8(inv);
  out[gid + out_offset / 8] = outv;
}

__kernel void copy(__global float8 * inout,
                   int in_offset)
{
  const int gid = get_global_id(0);
  inout[gid] = inout[gid + in_offset / 8];
}

__kernel void loadyuv_nhwc(__global uchar const * const Y,
                           __global uchar const * const U,
                           __global uchar const * const V,
                           __global float * out)
{
  const int gid = get_global_id(0);
  const int out_width = TRANSFORMED_WIDTH / 2;
  const int ox = gid % out_width;
  const int oy = gid / out_width;

  const int ix = ox * 2;
  const int iy = oy * 2;
  const int y_row0 = iy * TRANSFORMED_WIDTH;
  const int y_row1 = (iy + 1) * TRANSFORMED_WIDTH;
  const int base = gid * 6;

  out[base + 0] = (float)Y[y_row0 + ix];
  out[base + 1] = (float)Y[y_row1 + ix];
  out[base + 2] = (float)Y[y_row0 + ix + 1];
  out[base + 3] = (float)Y[y_row1 + ix + 1];
  out[base + 4] = (float)U[gid];
  out[base + 5] = (float)V[gid];
}

inline uchar warp_sample(__global uchar const * const src,
                         int src_step, int src_offset, int src_rows, int src_cols,
                         int dx, int dy, __constant float *M)
{
  const float X0 = M[0] * dx + M[1] * dy + M[2];
  const float Y0 = M[3] * dx + M[4] * dy + M[5];
  float W = M[6] * dx + M[7] * dy + M[8];
  W = W != 0.0f ? (float)INTER_TAB_SIZE / W : 0.0f;
  const int X = rint(X0 * W);
  const int Y = rint(Y0 * W);

  const short sx = convert_short_sat(X >> INTER_BITS);
  const short sy = convert_short_sat(Y >> INTER_BITS);
  const short ay = (short)(Y & (INTER_TAB_SIZE - 1));
  const short ax = (short)(X & (INTER_TAB_SIZE - 1));

  const int v0 = (sx >= 0 && sx < src_cols && sy >= 0 && sy < src_rows) ?
    convert_int(src[mad24(sy, src_step, src_offset + sx)]) : 0;
  const int v1 = (sx + 1 >= 0 && sx + 1 < src_cols && sy >= 0 && sy < src_rows) ?
    convert_int(src[mad24(sy, src_step, src_offset + (sx + 1))]) : 0;
  const int v2 = (sx >= 0 && sx < src_cols && sy + 1 >= 0 && sy + 1 < src_rows) ?
    convert_int(src[mad24(sy + 1, src_step, src_offset + sx)]) : 0;
  const int v3 = (sx + 1 >= 0 && sx + 1 < src_cols && sy + 1 >= 0 && sy + 1 < src_rows) ?
    convert_int(src[mad24(sy + 1, src_step, src_offset + (sx + 1))]) : 0;

  const float taby = 1.f / INTER_TAB_SIZE * ay;
  const float tabx = 1.f / INTER_TAB_SIZE * ax;
  const int itab0 = convert_short_sat_rte((1.0f - taby) * (1.0f - tabx) * INTER_REMAP_COEF_SCALE);
  const int itab1 = convert_short_sat_rte((1.0f - taby) * tabx * INTER_REMAP_COEF_SCALE);
  const int itab2 = convert_short_sat_rte(taby * (1.0f - tabx) * INTER_REMAP_COEF_SCALE);
  const int itab3 = convert_short_sat_rte(taby * tabx * INTER_REMAP_COEF_SCALE);

  const int val = v0 * itab0 + v1 * itab1 + v2 * itab2 + v3 * itab3;
  return convert_uchar_sat((val + (1 << (INTER_REMAP_COEF_BITS - 1))) >> INTER_REMAP_COEF_BITS);
}

__kernel void warp_loadyuv_nhwc(__global uchar const * const yuv,
                                int in_y_width, int in_y_height,
                                int in_uv_width, int in_uv_height,
                                int in_y_offset, int in_u_offset, int in_v_offset,
                                __global float *out,
                                __constant float *M_y,
                                __constant float *M_uv)
{
  const int ox = get_global_id(0);
  const int oy = get_global_id(1);
  const int out_width = TRANSFORMED_WIDTH / 2;
  const int out_height = TRANSFORMED_HEIGHT / 2;
  if (ox >= out_width || oy >= out_height) return;

  const int ix = ox * 2;
  const int iy = oy * 2;
  const int gid = oy * out_width + ox;
  const int base = gid * 6;

  out[base + 0] = (float)warp_sample(yuv, in_y_width, in_y_offset, in_y_height, in_y_width, ix,     iy,     M_y);
  out[base + 1] = (float)warp_sample(yuv, in_y_width, in_y_offset, in_y_height, in_y_width, ix,     iy + 1, M_y);
  out[base + 2] = (float)warp_sample(yuv, in_y_width, in_y_offset, in_y_height, in_y_width, ix + 1, iy,     M_y);
  out[base + 3] = (float)warp_sample(yuv, in_y_width, in_y_offset, in_y_height, in_y_width, ix + 1, iy + 1, M_y);
  out[base + 4] = (float)warp_sample(yuv, in_uv_width, in_u_offset, in_uv_height, in_uv_width, ox, oy, M_uv);
  out[base + 5] = (float)warp_sample(yuv, in_uv_width, in_v_offset, in_uv_height, in_uv_width, ox, oy, M_uv);
}
