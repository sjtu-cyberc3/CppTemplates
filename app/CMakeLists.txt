add_library(app_example src/app_example.cc)
add_library(app::example ALIAS app_example)

target_include_directories(app_example PUBLIC include)
target_link_libraries(app_example modules::example)
