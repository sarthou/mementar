cmake_minimum_required(VERSION 3.5)
project(mementar)

add_compile_options(-std=c++14 -Wall)

include_directories(
  ../../../../include
)

add_library(mementar_compression_lib
  binaryManagement/BitFileGenerator.cpp
  binaryManagement/BitFileGetter.cpp
  compressing/LzCompress.cpp
  compressing/LzUncompress.cpp
  compressing/Huffman.cpp
  compressing/Huffman_old.cpp
  archiving/Header.cpp
  archiving/Archive.cpp
)
target_link_libraries(mementar_compression_lib pthread)

add_executable(compress tests/compress.cpp)
target_link_libraries(compress mementar_compression_lib)

add_executable(uncompress tests/uncompress.cpp)
target_link_libraries(uncompress mementar_compression_lib)

add_executable(archive tests/archive.cpp)
target_link_libraries(archive mementar_compression_lib)

# ##
add_executable(huffman_in tests/perfs/huffman_in.cpp)
target_link_libraries(huffman_in mementar_compression_lib)

add_executable(huffman_out tests/perfs/huffman_out.cpp)
target_link_libraries(huffman_out mementar_compression_lib)

add_executable(lz_in tests/perfs/lz_in.cpp)
target_link_libraries(lz_in mementar_compression_lib)

add_executable(lz_out tests/perfs/lz_out.cpp)
target_link_libraries(lz_out mementar_compression_lib)
