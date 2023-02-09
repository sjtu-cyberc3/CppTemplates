#pragma once

#include "core.h"
#include <yaml-cpp/yaml.h>

namespace YAML {
template <typename T> struct convert<T, std::enable_if_t<reflect::is_reflectable_v<T>, void>> {
  static Node encode(const T& obj) {
    Node node;
    reflect::public_foreach_recursive<T>([&](auto info) { node[info.name().data()] = obj.*info.ptr(); });
    return node;
  }

  static bool decode(const Node& node, T& obj) {
    reflect::public_foreach_recursive<T>(
      [&](auto info) { obj.*info.ptr() = node[info.name().data()].template as<typename decltype(info)::type>(); });
    return true;
  }
};
}  // namespace YAML