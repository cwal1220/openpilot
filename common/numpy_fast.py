from bisect import bisect_left

try:
  from common.numpy_fast_pyx import clip, copy_to_f64, copy_to_i64, interp, interp_to_f64, mean
except ImportError:

  def clip(x, lo, hi):
    y = x if x < hi else hi
    return y if y > lo else lo

  def _interp_scalar(xv, xp, fp, N):
    hi = bisect_left(xp, xv)
    low = hi - 1
    return fp[-1] if hi == N and xv > xp[low] else (
      fp[0] if hi == 0 else
      (xv - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low])

  def interp(x, xp, fp):
    N = len(xp)
    return [_interp_scalar(v, xp, fp, N) for v in x] if hasattr(x, '__iter__') else _interp_scalar(x, xp, fp, N)

  def interp_to_f64(x, xp, fp, out, count):
    N = len(xp)
    for i in range(count):
      out[i] = _interp_scalar(x[i], xp, fp, N)

  def mean(x):
    return sum(x) / len(x)

  def copy_to_f64(src, dst, count):
    for i in range(count):
      dst[i] = src[i]

  def copy_to_i64(src, dst, count):
    for i in range(count):
      dst[i] = int(src[i])
