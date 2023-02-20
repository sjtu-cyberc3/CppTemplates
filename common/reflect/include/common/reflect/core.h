#pragma once

#include <boost/preprocessor.hpp>

// clang-format off

#ifndef REFLECT_YAML
#define REFLECT_YAML(...)
#endif

#define STRUCT_INFO(C, Bases, Public) CLASS_INFO(C, Bases, Public, , )
#define CLASS_INFO(C, Bases, Public, Protected, Private)                                                                                            \
  template <> constexpr inline bool reflect::is_reflectable_v<C> = true;                                                                            \
  template <> constexpr auto        reflect::ClassInfo<C>::base_infos() { return std::make_tuple(BOOST_PP_SEQ_FOR_EACH_I(BASE_INFO, C, Bases)); }   \
  template <> constexpr auto        reflect::ClassInfo<C>::public_infos() { return std::make_tuple(BOOST_PP_SEQ_FOR_EACH_I(VAR_INFO, C, Public)); } \
  template <> constexpr auto reflect::ClassInfo<C>::protected_infos() { return std::make_tuple(BOOST_PP_SEQ_FOR_EACH_I(VAR_INFO, C, Protected)); }  \
  template <> constexpr auto reflect::ClassInfo<C>::private_infos() { return std::make_tuple(BOOST_PP_SEQ_FOR_EACH_I(VAR_INFO, C, Private)); }      \
  REFLECT_YAML(C)

#define BASE_INFO(r, c, i, b)                                             \
  BOOST_PP_COMMA_IF(i)[]() {                                              \
    constexpr struct {                                                    \
      using type = b;                                                     \
      constexpr std::string_view name() { return BOOST_PP_STRINGIZE(b); } \
    } info;                                                               \
    return info;                                                          \
  }                                                                       \
  ()

#define VAR_INFO(r, c, i, v)                                              \
  BOOST_PP_COMMA_IF(i)[]() {                                              \
    constexpr struct {                                                    \
      using type = reflect::internal::member_type_t<decltype(&c::v)>;     \
      constexpr std::string_view name() { return BOOST_PP_STRINGIZE(v); } \
      constexpr auto             ptr() { return &c::v; }                  \
    } info;                                                               \
    return info;                                                          \
  }                                                                       \
  ()

#include <functional>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

namespace reflect::internal {
template <typename T> struct member_type {
  using type = T;
};
template <typename C, typename T> struct member_type<T C::*> {
  using type = T;
};
template <typename T> using member_type_t = typename member_type<T>::type;

template <typename T, typename F, size_t... Idx> constexpr void tuple_foreach_impl(const T& tuple, F&& func, std::index_sequence<Idx...>) {
  (std::invoke(std::forward<F>(func), std::get<Idx>(tuple)), ...);
}
template <typename T, typename F> constexpr void tuple_foreach(const T& tuple, F&& func) {
  tuple_foreach_impl(tuple, func, std::make_index_sequence<std::tuple_size_v<T>>{});
}
}  // namespace reflect::internal

namespace reflect {
template <typename T> constexpr bool is_reflectable_v = false;
template <typename T> struct c : T {
  static_assert(is_reflectable_v<T>);
};
template <typename T> struct ClassInfo {
  static_assert(is_reflectable_v<T>);
  static constexpr auto base_infos(){};
  static constexpr auto public_infos(){};
  static constexpr auto protected_infos(){};
  static constexpr auto private_infos(){};
};

template <typename T, typename F> constexpr std::enable_if_t<is_reflectable_v<T>, void> base_foreach(F&& func) {
  internal::tuple_foreach(ClassInfo<T>::base_infos(), std::forward<F>(func));
}

template <typename T, typename F> constexpr std::enable_if_t<is_reflectable_v<T>, void> public_foreach(F&& func) {
  internal::tuple_foreach(ClassInfo<T>::public_infos(), std::forward<F>(func));
}

template <typename T, typename F> constexpr std::enable_if_t<is_reflectable_v<T>, void> protected_foreach(F&& func) {
  internal::tuple_foreach(ClassInfo<T>::protected_infos(), std::forward<F>(func));
}

template <typename T, typename F> constexpr std::enable_if_t<is_reflectable_v<T>, void> private_foreach(F&& func) {
  internal::tuple_foreach(ClassInfo<T>::private_infos(), std::forward<F>(func));
}

template <typename T, typename F> constexpr std::enable_if_t<is_reflectable_v<T>, void> variable_foreach(F&& func) {
  public_foreach<T>(std::forward<F>(func));
  protected_foreach<T>(std::forward<F>(func));
  private_foreach<T>(std::forward<F>(func));
}

template <typename T, typename F> constexpr std::enable_if_t<is_reflectable_v<T>, void> base_foreach_recursive(F&& func) {
  internal::tuple_foreach(ClassInfo<T>::base_infos(), [&](auto info) {
    std::invoke(std::forward<F>(func), info);
    if constexpr (is_reflectable_v<typename decltype(info)::type>) {
      base_foreach_recursive<typename decltype(info)::type>(std::forward<F>(func));
    }
  });
}

template <typename T, typename F> constexpr std::enable_if_t<is_reflectable_v<T>, void> public_foreach_recursive(F&& func) {
  internal::tuple_foreach(ClassInfo<T>::public_infos(), std::forward<F>(func));
  base_foreach_recursive<T>([&](auto info) {
    if constexpr (is_reflectable_v<typename decltype(info)::type>) {
      public_foreach<typename decltype(info)::type>(std::forward<F>(func));
    }
  });
}

template <typename T, typename F> constexpr std::enable_if_t<is_reflectable_v<T>, void> protected_foreach_recursive(F&& func) {
  internal::tuple_foreach(ClassInfo<T>::protected_infos(), std::forward<F>(func));
  base_foreach_recursive<T>([&](auto info) {
    if constexpr (is_reflectable_v<typename decltype(info)::type>) {
      protected_foreach<typename decltype(info)::type>(std::forward<F>(func));
    }
  });
}

template <typename T, typename F> constexpr std::enable_if_t<is_reflectable_v<T>, void> private_foreach_recursive(F&& func) {
  internal::tuple_foreach(ClassInfo<T>::private_infos(), std::forward<F>(func));
  base_foreach_recursive<T>([&](auto info) {
    if constexpr (is_reflectable_v<typename decltype(info)::type>) {
      private_foreach<typename decltype(info)::type>(std::forward<F>(func));
    }
  });
}

template <typename T, typename F> constexpr std::enable_if_t<is_reflectable_v<T>, void> variable_foreach_recursive(F&& func) {
  public_foreach_recursive<T>(std::forward<F>(func));
  protected_foreach_recursive<T>(std::forward<F>(func));
  private_foreach_recursive<T>(std::forward<F>(func));
}

}  // namespace reflect