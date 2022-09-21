#pragma once

#include "mbed.h"

template <typename T>
T max_limiter(T value, T max)
{
  return (value <= max) ? value : max;
};
template <typename T>
T min_limiter(T value, T min)
{
  return (value >= min) ? value : min;
};
template <typename T>
T range_limiter(T value, T min, T max)
{
  if (max < min)
  {
    error("utility.hpp: range_limiter: min argument is bigger than max argument.");
  }
  if (value <= max)
  {
    if (value >= min)
    {
      return value;
    }
    else
    {
      return min;
    }
  }
  else
  {
    return max;
  }
}

template <typename T>
T abs_limiter(T value, T abs)
{
  if (abs < static_cast<T>(0))
  {
    error("utility.hpp: abs_limiter: abs argument is negative.");
  }
  return range_limiter<T>(value, -abs, abs);
}

template <typename T>
T sign(T value)
{
  if (value == static_cast<T>(0))
  {
    error("utility.hpp: sign: devide by 0");
  }
  return value / abs(value);
}