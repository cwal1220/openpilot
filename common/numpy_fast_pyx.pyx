# distutils: language = c++
# cython: language_level = 3
# cython: boundscheck=False, wraparound=False

cdef inline double _clip_double(double x, double lo, double hi):
  cdef double y = x if x < hi else hi
  return y if y > lo else lo

cdef inline double _interp_scalar_double(double xv, object xp, object fp, Py_ssize_t n):
  cdef Py_ssize_t lo = 0
  cdef Py_ssize_t hi = n
  cdef Py_ssize_t mid
  cdef Py_ssize_t low
  cdef double xp_mid
  cdef double xp_low
  cdef double xp_hi
  cdef double fp_low
  cdef double fp_hi

  while lo < hi:
    mid = (lo + hi) // 2
    xp_mid = xp[mid]
    if xp_mid < xv:
      lo = mid + 1
    else:
      hi = mid

  low = lo - 1
  if lo == n:
    xp_low = xp[low]
    if xv > xp_low:
      return fp[n - 1]
  elif lo == 0:
    return fp[0]

  xp_low = xp[low]
  xp_hi = xp[lo]
  fp_low = fp[low]
  fp_hi = fp[lo]
  return (xv - xp_low) * (fp_hi - fp_low) / (xp_hi - xp_low) + fp_low

cdef inline double _interp_scalar_f64(double xv, double[:] xp, double[:] fp, Py_ssize_t n):
  cdef Py_ssize_t lo = 0
  cdef Py_ssize_t hi = n
  cdef Py_ssize_t mid
  cdef Py_ssize_t low
  cdef double xp_low

  while lo < hi:
    mid = (lo + hi) // 2
    if xp[mid] < xv:
      lo = mid + 1
    else:
      hi = mid

  low = lo - 1
  if lo == n:
    xp_low = xp[low]
    if xv > xp_low:
      return fp[n - 1]
  elif lo == 0:
    return fp[0]

  return (xv - xp[low]) * (fp[lo] - fp[low]) / (xp[lo] - xp[low]) + fp[low]

def clip(double x, double lo, double hi):
  return _clip_double(x, lo, hi)

def interp(x, xp, fp):
  cdef Py_ssize_t n = len(xp)
  cdef list out
  cdef object v

  if hasattr(x, '__iter__'):
    out = []
    for v in x:
      out.append(_interp_scalar_double(v, xp, fp, n))
    return out
  return _interp_scalar_double(x, xp, fp, n)

def interp_to_f64(double[:] x, double[:] xp, double[:] fp, double[:] out, Py_ssize_t count):
  cdef Py_ssize_t n = len(xp)
  cdef Py_ssize_t i
  for i in range(count):
    out[i] = _interp_scalar_f64(x[i], xp, fp, n)

def mean(x):
  cdef Py_ssize_t n = len(x)
  cdef Py_ssize_t i
  cdef double total = 0.0
  for i in range(n):
    total += x[i]
  return total / n

def copy_to_f64(object src, double[:] dst, Py_ssize_t count):
  cdef Py_ssize_t i
  for i in range(count):
    dst[i] = src[i]

def copy_to_i64(object src, long[:] dst, Py_ssize_t count):
  cdef Py_ssize_t i
  for i in range(count):
    dst[i] = <long>src[i]
