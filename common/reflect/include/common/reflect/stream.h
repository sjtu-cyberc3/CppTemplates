#pragma once

#include "core.h"
#include <iostream>
#include <type_traits>

#define CALLONCE(flag) \
  if (flag) {          \
    flag = false;      \
  } else

template <typename T> std::enable_if_t<reflect::is_reflectable_v<T>, std::ostream&> operator<<(std::ostream& out, const T& obj) {
  bool first = true;
  reflect::base_foreach<T>([&](auto info) {
    CALLONCE(first) {
      out << " ";
    }
    out << static_cast<const typename decltype(info)::type&>(obj);
  });
  reflect::variable_foreach<T>([&](auto info) {
    CALLONCE(first) {
      out << " ";
    }
    out << (obj.*info.ptr());
  });
  return out;
}

template <typename T> std::enable_if_t<reflect::is_reflectable_v<T>, std::istream&> operator>>(std::istream& in, T& obj) {
  reflect::base_foreach<T>([&](auto info) { in >> static_cast<typename decltype(info)::type&>(obj); });
  reflect::variable_foreach<T>([&](auto info) { in >> (obj.*info.ptr()); });
  return in;
}
