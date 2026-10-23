# PC simulator: renders the model with the exact firmware renderer into PPM frames.
#   cmake -S sim -B sim/build && cmake --build sim/build
#   sim/build/r3d_sim firmware/models/model.bin out/frame_ 456 280 0 36 215 10 16 1 1
cmake_minimum_required(VERSION 3.16)
project(r3d_sim C)
set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(R3D ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/components/r3d)
add_executable(r3d_sim sim.c ${R3D}/r3d.c ${R3D}/r3d_model.c)
target_include_directories(r3d_sim PRIVATE ${R3D}/include)
if(NOT MSVC)
  target_link_libraries(r3d_sim m)
endif()
