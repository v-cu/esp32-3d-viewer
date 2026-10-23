cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-3d-viewer)

# `idf.py flash` also writes the 3D model into the "model" partition,
# `idf.py model-flash` writes only the model (fast swap, no rebuild needed).
set(CAR3D_MODEL_BIN "${CMAKE_CURRENT_SOURCE_DIR}/models/model.bin" CACHE FILEPATH "R3DM model image")
if(EXISTS "${CAR3D_MODEL_BIN}")
    esptool_py_flash_to_partition(flash "model" "${CAR3D_MODEL_BIN}")
    esptool_py_custom_target(model-flash model "partition_table_bin")
    esptool_py_flash_to_partition(model-flash "model" "${CAR3D_MODEL_BIN}")
else()
    message(WARNING "No model at ${CAR3D_MODEL_BIN} - run tools/model_convert.py first")
endif()
