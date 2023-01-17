#pragma once

#include "reflect.h"
#include <fmt/format.h>

template <typename T> struct fmt::formatter<T, char, std::enable_if_t<reflect::is_reflectable_v<T>, void>> {
  constexpr auto parse(format_parse_context& ctx) {
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && *it == 'p') {
      presentation = *it++;
    }
    while (it != end && *it != '}' && presentation == 'p' && '0' <= *it && *it <= '9') {
      indent = indent * 10 + *it++ - '0';
    }
    if (it != end && *it != '}') {
      throw fmt::format_error("invalid format");
    }
    return it;
  }

  template <typename FormatContext> constexpr auto format(const T& obj, FormatContext& ctx) {
    if (presentation == 'p') {
      fmt::format_to(ctx.out(), "{{\n");
      auto func = [&](auto info) {
        for (int i = 0; i < indent + 1; i++)
          fmt::format_to(ctx.out(), "  ");
        fmt::format_to(ctx.out(), "{} = ", info.name());
        if constexpr (reflect::is_reflectable_v<typename decltype(info)::type>) {
          std::string fmt_str = fmt::format("{{:p{}}},\n", indent + 1);
          fmt::format_to(ctx.out(), fmt_str, obj.*(info.ptr()), indent + 1);
        } else {
          fmt::format_to(ctx.out(), "{},\n", obj.*(info.ptr()));
        }
      };
      reflect::public_foreach_recursive<T>(func);
      reflect::protected_foreach_recursive<T>(func);
      reflect::private_foreach_recursive<T>(func);
      for (int i = 0; i < indent; i++)
        fmt::format_to(ctx.out(), "  ");
      return fmt::format_to(ctx.out(), "}}");
    } else {
      fmt::format_to(ctx.out(), "{{");
      auto func = [&](auto info) { fmt::format_to(ctx.out(), "{} = {}, ", info.name(), obj.*(info.ptr())); };
      reflect::public_foreach_recursive<T>(func);
      reflect::protected_foreach_recursive<T>(func);
      reflect::private_foreach_recursive<T>(func);
      return fmt::format_to(ctx.out(), "}}");
    }
  }

  char presentation = 0;
  int  indent       = 0;
};
