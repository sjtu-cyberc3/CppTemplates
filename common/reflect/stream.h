#pragma once

#include "reflect.h"
#include <iostream>
#include <type_traits>

template <typename T> std::enable_if_t<reflect::is_reflectable_v<T>, std::ostream&> operator<<(std::ostream& out, const T& obj) {
  reflect::public_foreach_recursive<T>([&](auto info) { out << obj.*(info.ptr()) << " "; });
  reflect::protected_foreach_recursive<T>([&](auto info) { out << obj.*(info.ptr()) << " "; });
  reflect::private_foreach_recursive<T>([&](auto info) { out << obj.*(info.ptr()) << " "; });
  return out;
}

template <typename T> std::enable_if_t<reflect::is_reflectable_v<T>, std::istream&> operator>>(std::istream& in, T& obj) {
  reflect::public_foreach_recursive<T>([&](auto info) { in >> obj.*(info.ptr()); });
  reflect::protected_foreach_recursive<T>([&](auto info) { in >> obj.*(info.ptr()); });
  reflect::private_foreach_recursive<T>([&](auto info) { in >> obj.*(info.ptr()); });
  return in;
}
