cmake_minimum_required(VERSION 3.10)
project(nanopb)

file(GLOB_RECURSE NANOPB_SRC src/*.c)
add_library(nanopb STATIC ${NANOPB_SRC})

target_include_directories(nanopb PRIVATE include/nanopb)
target_include_directories(nanopb PUBLIC include)