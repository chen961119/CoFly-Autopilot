#pragma once

#include <cmath>
#include <cstdint>

template <typename T, typename L, typename H>
constexpr auto constrain(T value, L minimum, H maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}
