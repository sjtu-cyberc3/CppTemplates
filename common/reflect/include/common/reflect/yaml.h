#pragma once

#include "core.h"
#include <yaml-cpp/yaml.h>

#ifdef REFLECT_YAML
#undef REFLECT_YAML
#endif
#define REFLECT_YAML(Class)                                                    \
  namespace YAML {                                                             \
  template <> struct convert<Class> {                                          \
    static Node encode(const Class &obj) {                                     \
      Node node;                                                               \
      reflect::public_foreach_recursive<Class>(                                \
          [&](auto info) { node[info.name().data()] = obj.*info.ptr(); });     \
      return node;                                                             \
    }                                                                          \
    static bool decode(const Node &node, Class &obj) {                         \
      reflect::public_foreach_recursive<Class>([&](auto info) {                \
        obj.*info.ptr() = node[info.name().data()]                             \
                              .template as<typename decltype(info)::type>();   \
      });                                                                      \
      return true;                                                             \
    }                                                                          \
  };                                                                           \
  }
