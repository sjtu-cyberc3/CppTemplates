#include "fmt.h"
#include "reflect.h"
#include "stream.h"
#include "yaml.h"

#include <gtest/gtest.h>
#include <sstream>

struct A {
  int a;
};
STRUCT_INFO(A, , (a))

struct B : A {
  int b;
};
STRUCT_INFO(B, (A), (b))

struct C : B {
  int c;
};
STRUCT_INFO(C, (B), (c))

struct D {
  A a;
  B b;
  C c;
};
STRUCT_INFO(D, , (a)(b)(c))

class E {
  friend struct reflect::ClassInfo<E>;

  friend class reflect_stream_e_Test;
  friend class reflect_fmt_e_Test;

 private:
  int x, y, z;
};
CLASS_INFO(E, , , , (x)(y)(z))

TEST(reflect, reflect_a) {
  int base_cnt, public_cnt;

  base_cnt = 0;
  reflect::base_foreach<A>([&]([[maybe_unused]] auto info) { FAIL(); });
  EXPECT_EQ(base_cnt, 0);

  base_cnt = 0;
  reflect::base_foreach_recursive<A>([&]([[maybe_unused]] auto info) { FAIL(); });
  EXPECT_EQ(base_cnt, 0);

  public_cnt = 0;
  reflect::public_foreach<A>([&](auto info) {
    if constexpr (info.name() == "a") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, int>));
      public_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(public_cnt, 1);

  public_cnt = 0;
  reflect::public_foreach_recursive<A>([&](auto info) {
    if constexpr (info.name() == "a") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, int>));
      EXPECT_EQ(info.ptr(), &A::a);
      public_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(public_cnt, 1);
}

TEST(reflect, reflect_b) {
  int base_cnt, public_cnt;

  base_cnt = 0;
  reflect::base_foreach<B>([&](auto info) {
    if constexpr (info.name() == "A") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, A>));
      base_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(base_cnt, 1);

  base_cnt = 0;
  reflect::base_foreach_recursive<B>([&](auto info) {
    if constexpr (info.name() == "A") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, A>));
      base_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(base_cnt, 1);

  public_cnt = 0;
  reflect::public_foreach<B>([&](auto info) {
    if constexpr (info.name() == "b") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, int>));
      EXPECT_EQ(info.ptr(), &B::b);
      public_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(public_cnt, 1);

  public_cnt = 0;
  reflect::public_foreach_recursive<B>([&](auto info) {
    if constexpr (info.name() == "a") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, int>));
      EXPECT_EQ(info.ptr(), &B::a);
      public_cnt++;
    } else if constexpr (info.name() == "b") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, int>));
      EXPECT_EQ(info.ptr(), &B::b);
      public_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(public_cnt, 2);
}

TEST(reflect, reflect_c) {
  int base_cnt, public_cnt;

  base_cnt = 0;
  reflect::base_foreach<C>([&](auto info) {
    if constexpr (info.name() == "B") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, B>));
      base_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(base_cnt, 1);

  base_cnt = 0;
  reflect::base_foreach_recursive<C>([&](auto info) {
    if constexpr (info.name() == "A") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, A>));
      base_cnt++;
    } else if constexpr (info.name() == "B") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, B>));
      base_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(base_cnt, 2);

  public_cnt = 0;
  reflect::public_foreach<C>([&](auto info) {
    if constexpr (info.name() == "c") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, int>));
      EXPECT_EQ(info.ptr(), &C::c);
      public_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(public_cnt, 1);

  public_cnt = 0;
  reflect::public_foreach_recursive<C>([&](auto info) {
    if constexpr (info.name() == "a") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, int>));
      EXPECT_EQ(info.ptr(), &C::a);
      public_cnt++;
    } else if constexpr (info.name() == "b") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, int>));
      EXPECT_EQ(info.ptr(), &C::b);
      public_cnt++;
    } else if constexpr (info.name() == "c") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, int>));
      EXPECT_EQ(info.ptr(), &C::c);
      public_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(public_cnt, 3);
}

TEST(reflect, reflect_d) {
  int base_cnt, public_cnt;

  base_cnt = 0;
  reflect::base_foreach<D>([&]([[maybe_unused]] auto info) { FAIL(); });
  EXPECT_EQ(base_cnt, 0);

  base_cnt = 0;
  reflect::base_foreach_recursive<D>([&]([[maybe_unused]] auto info) { FAIL(); });
  EXPECT_EQ(base_cnt, 0);

  public_cnt = 0;
  reflect::public_foreach<D>([&](auto info) {
    if constexpr (info.name() == "a") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, A>));
      EXPECT_EQ(info.ptr(), &D::a);
      public_cnt++;
    } else if constexpr (info.name() == "b") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, B>));
      EXPECT_EQ(info.ptr(), &D::b);
      public_cnt++;
    } else if constexpr (info.name() == "c") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, C>));
      EXPECT_EQ(info.ptr(), &D::c);
      public_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(public_cnt, 3);

  public_cnt = 0;
  reflect::public_foreach_recursive<D>([&](auto info) {
    if constexpr (info.name() == "a") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, A>));
      EXPECT_EQ(info.ptr(), &D::a);
      public_cnt++;
    } else if constexpr (info.name() == "b") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, B>));
      EXPECT_EQ(info.ptr(), &D::b);
      public_cnt++;
    } else if constexpr (info.name() == "c") {
      EXPECT_TRUE((std::is_same_v<typename decltype(info)::type, C>));
      EXPECT_EQ(info.ptr(), &D::c);
      public_cnt++;
    } else {
      FAIL();
    }
  });
  EXPECT_EQ(public_cnt, 3);
}

TEST(reflect, stream_a) {
  std::stringstream ss;
  A                 a_out, a_in;
  a_out.a = 1;
  ss << a_out;
  EXPECT_EQ(ss.str(), "1 ");
  ss >> a_in;
  EXPECT_EQ(a_out.a, a_in.a);
}

TEST(reflect, stream_b) {
  std::stringstream ss;
  B                 b_out, b_in;
  b_out.a = 1;
  b_out.b = 2;
  ss << b_out;
  EXPECT_EQ(ss.str(), "2 1 ");
  ss >> b_in;
  EXPECT_EQ(b_in.a, b_out.a);
  EXPECT_EQ(b_in.b, b_out.b);
}

TEST(reflect, stream_c) {
  std::stringstream ss;
  C                 c_out, c_in;
  c_out.a = 1;
  c_out.b = 2;
  c_out.c = 3;
  ss << c_out;
  EXPECT_EQ(ss.str(), "3 2 1 ");
  ss >> c_in;
  EXPECT_EQ(c_in.a, c_out.a);
  EXPECT_EQ(c_in.b, c_out.b);
  EXPECT_EQ(c_in.c, c_out.c);
}

TEST(reflect, stream_d) {
  std::stringstream ss;
  D                 d_out, d_in;
  d_out.a.a = 1;
  d_out.b.a = 2;
  d_out.b.b = 3;
  d_out.c.a = 4;
  d_out.c.b = 5;
  d_out.c.c = 6;
  ss << d_out;
  EXPECT_EQ(ss.str(), "1  3 2  6 5 4  ");
  ss >> d_in;
  EXPECT_EQ(d_in.a.a, d_out.a.a);
  EXPECT_EQ(d_in.b.a, d_out.b.a);
  EXPECT_EQ(d_in.b.b, d_out.b.b);
  EXPECT_EQ(d_in.c.a, d_out.c.a);
  EXPECT_EQ(d_in.c.b, d_out.c.b);
  EXPECT_EQ(d_in.c.c, d_out.c.c);
}

TEST(reflect, stream_e) {
  E e_out, e_in;
  e_out.x = 1;
  e_out.y = 2;
  e_out.z = 3;
  std::stringstream ss;
  ss << e_out;
  EXPECT_EQ(ss.str(), "1 2 3 ");
  ss >> e_in;
}

TEST(reflect, fmt_a) {
  A a;
  a.a             = 1;
  std::string str = fmt::format("{}", a);
  EXPECT_EQ(str, "{a = 1, }");
  std::string str_pretty = fmt::format("{:p}", a);
  EXPECT_EQ(str_pretty, R"({
  a = 1,
})");
}

TEST(reflect, fmt_b) {
  B b;
  b.a             = 1;
  b.b             = 2;
  std::string str = fmt::format("{}", b);
  EXPECT_EQ(str, "{b = 2, a = 1, }");
  std::string pretty_str = fmt::format("{:p}", b);
  EXPECT_EQ(pretty_str, R"({
  b = 2,
  a = 1,
})");
}

TEST(reflect, fmt_c) {
  C c;
  c.a             = 1;
  c.b             = 2;
  c.c             = 3;
  std::string str = fmt::format("{}", c);
  EXPECT_EQ(str, "{c = 3, b = 2, a = 1, }");
  std::string pretty_str = fmt::format("{:p}", c);
  EXPECT_EQ(pretty_str, R"({
  c = 3,
  b = 2,
  a = 1,
})");
}

TEST(reflect, fmt_d) {
  D d;
  d.a.a           = 1;
  d.b.a           = 2;
  d.b.b           = 3;
  d.c.a           = 4;
  d.c.b           = 5;
  d.c.c           = 6;
  std::string str = fmt::format("{}", d);
  EXPECT_EQ(str, "{a = {a = 1, }, b = {b = 3, a = 2, }, c = {c = 6, b = 5, a = 4, }, }");
  std::string pretty_str = fmt::format("{:p}", d);
  EXPECT_EQ(pretty_str, R"({
  a = {
    a = 1,
  },
  b = {
    b = 3,
    a = 2,
  },
  c = {
    c = 6,
    b = 5,
    a = 4,
  },
})");
}

TEST(reflect, fmt_e) {
  E e;
  e.x             = 1;
  e.y             = 2;
  e.z             = 3;
  std::string str = fmt::format("{}", e);
  EXPECT_EQ(str, "{x = 1, y = 2, z = 3, }");
  std::string str_pretty = fmt::format("{:p}", e);
  EXPECT_EQ(str_pretty, R"({
  x = 1,
  y = 2,
  z = 3,
})");
}

TEST(reflect, yaml_a) {
  A a_out, a_in;
  a_out.a = 1;
  YAML::Node node;
  node = a_out;
  std::stringstream ss;
  ss << node;
  node = YAML::Load(ss.str());
  a_in = node.as<A>();
  EXPECT_EQ(a_in.a, a_out.a);
}

TEST(reflect, yaml_b) {
  B b_out, b_in;
  b_out.a = 1;
  b_out.b = 2;
  YAML::Node node;
  node = b_out;
  std::stringstream ss;
  ss << node;
  node = YAML::Load(ss.str());
  b_in = node.as<B>();
  EXPECT_EQ(b_in.a, b_out.a);
  EXPECT_EQ(b_in.b, b_out.b);
}

TEST(reflect, yaml_c) {
  C c_out, c_in;
  c_out.a = 1;
  c_out.b = 2;
  c_out.c = 3;
  YAML::Node node;
  node = c_out;
  std::stringstream ss;
  ss << node;
  node = YAML::Load(ss.str());
  c_in = node.as<C>();
  EXPECT_EQ(c_in.a, c_out.a);
  EXPECT_EQ(c_in.b, c_out.b);
  EXPECT_EQ(c_in.c, c_out.c);
}

TEST(reflect, yaml_d) {
    D                 d_out, d_in;
  d_out.a.a = 1;
  d_out.b.a = 2;
  d_out.b.b = 3;
  d_out.c.a = 4;
  d_out.c.b = 5;
  d_out.c.c = 6;
  YAML::Node node;
  node = d_out;
  std::stringstream ss;
  ss << node;
  node = YAML::Load(ss.str());
  d_in = node.as<D>();
  EXPECT_EQ(d_in.a.a, d_out.a.a);
  EXPECT_EQ(d_in.b.a, d_out.b.a);
  EXPECT_EQ(d_in.b.b, d_out.b.b);
  EXPECT_EQ(d_in.c.a, d_out.c.a);
  EXPECT_EQ(d_in.c.b, d_out.c.b);
  EXPECT_EQ(d_in.c.c, d_out.c.c);
}
