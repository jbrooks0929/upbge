/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#ifndef __UTIL_TYPES_H__
#  error "Do not include this file directly, include util/types.h instead."
#endif

CCL_NAMESPACE_BEGIN

#ifndef __KERNEL_NATIVE_VECTOR_TYPES__
#  ifdef __KERNEL_SSE__
__forceinline float4::float4()
{
}

__forceinline float4::float4(const __m128 &a) : m128(a)
{
}

__forceinline float4::operator const __m128 &() const
{
  return m128;
}

__forceinline float4::operator __m128 &()
{
  return m128;
}

__forceinline float4 &float4::operator=(const float4 &a)
{
  m128 = a.m128;
  return *this;
}
#  endif /* __KERNEL_SSE__ */

#  ifndef __KERNEL_GPU__
__forceinline float float4::operator[](int i) const
{
  util_assert(i >= 0);
  util_assert(i < 4);
  return *(&x + i);
}

__forceinline float &float4::operator[](int i)
{
  util_assert(i >= 0);
  util_assert(i < 4);
  return *(&x + i);
}
#  endif

ccl_device_inline float4 make_float4(float f)
{
#  ifdef __KERNEL_SSE__
  float4 a(_mm_set1_ps(f));
#  else
  float4 a = {f, f, f, f};
#  endif
  return a;
}

ccl_device_inline float4 make_float4(float x, float y, float z, float w)
{
#  ifdef __KERNEL_SSE__
  float4 a(_mm_set_ps(w, z, y, x));
#  else
  float4 a = {x, y, z, w};
#  endif
  return a;
}

ccl_device_inline float4 make_float4(const int4 &i)
{
#  ifdef __KERNEL_SSE__
  float4 a(_mm_cvtepi32_ps(i.m128));
#  else
  float4 a = {(float)i.x, (float)i.y, (float)i.z, (float)i.w};
#  endif
  return a;
}

ccl_device_inline void print_float4(const char *label, const float4 &a)
{
  printf("%s: %.8f %.8f %.8f %.8f\n", label, (double)a.x, (double)a.y, (double)a.z, (double)a.w);
}
#endif /* __KERNEL_NATIVE_VECTOR_TYPES__ */

CCL_NAMESPACE_END
