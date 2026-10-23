idf_component_register(
    SRCS "r3d.c" "r3d_model.c"
    INCLUDE_DIRS "include"
)
# the rasterizer is the hot path: always optimise it for speed
target_compile_options(${COMPONENT_LIB} PRIVATE -O3 -ffast-math -fno-math-errno)
