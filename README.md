# C++编程模版

这是一个C++编程模版，用于提高程序开发与调试效率。本部分与vscode结合紧密，强烈建议大家使用vscode作为首选IDE。

- [C++编程模版](#C++编程模版)
- [背景](#背景)
- [插件选择](#插件选择)
- [使用注意事项](#使用注意事项)
    - [moduels编写](#moduels编写)
    - [Doxygen编写](#Doxygen编写)
    - [命名规范](#命名规范)
    - [注释规范](#注释规范)
    - [结构划分](#结构划分)
    - [提交](#提交)
    - [依赖规范](#依赖规范)
- [TODO](#TODO)

# 背景

在以往的开发中，你可能一直在使用 **cout大法** 来进行程序错误的调试，苦于调试效率的低下；可能在网上搜索各种配置方法来尝试使用调试工具，但苦于逢debug必先修改配置，影响调试心情💢。在ros相关的开发中，你可能苦于官方cmakelist文件不够清晰，导致迁移编译时总是出现链接问题，而不知如何解决。在程序具体开发中，可能结构混乱不利于二次开发或子模块复用。在交付程序时，可能因为没有文档而造成沟通复杂。为了解决以上问题，我们规范了一种内部使用的C++编程模版，大家在使用中可以按照本模版建议方法进行调试与开发，逐步解决上述问题。

# 插件选择

请在vscode中安装如下插件，

1. cmake & cmake tools
2. Doxygen Documentation Generator
3. C/C++
4. Output Colozier，用于将输出信息颜色化，易于读取
5. C++ TestMate，用于支持测试示例level的调试
4. clang-format，用于c++程序格式化
5. cmake-format，用于cmakelist格式化

# 使用注意事项

## moduels编写
    1. modules用于存放各功能无关独立子模块
    2. 子模块均要求打包为类，以方面进行掉用
    3. 各子模块，均需构造 xxx__test.cc 进行完备功能测试，参考示例链接 gtest 库
    4. 若子模块需要测试数据，请创建 data 文件夹并在其中读取

## Doxygen编写
    1. 请安装Doxygen 注释自动生成插件
    2. 所有编写 .h .cc .cpp文件均需要生成 文件注释信息
    3. .h 中需生成 类及方法、成员的注释信息，具体请参考示例

## 命名规范
    1. 本部分规范完全参照[google style](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/naming/) 下面进行说明
    2. 文件命名规则： 同google style ， 另补充以下约定： 以下划线进行分割，对于含main函数的源文件，以.cpp文件结尾
    3. 类命名规则: 同google style
    4. 变量命名：同google style，具体项约定为 小写+下划线。
    5. 常量命名： 同google style
    5. 函数命名： 同google style
    6. 命名空间命名： 项目名+层次+具体模块， eg: namespace templates::modules::submodule1

## 注释规范
    1. 统一使用doxygen插件生成
    2. .cc 和 .cpp 文件只需进行文件注释
    3. .h 需进行文件，类，成员注释

## 结构划分
    1. modules用于存放功能子模块，或者消息类。不涉及具体业务流程实现
    2. app中存放业务流程相关信息，用于将各modules拼接起来，以实现最终功能

## 提交
    1. 请保证每一次提交代码，修改部分均经过format

## 依赖规范

1. 引入层与包的概念，层间只允许上游依赖下游，层内可相互依赖

# TODO

[] 补充app示例

[] 支持ros进入本结构