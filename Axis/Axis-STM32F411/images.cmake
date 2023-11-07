# Special Python package
find_package(Python3 REQUIRED)
find_file(IMAGE_CONVERTER_PY
        image_to_h.py
        PATHS ${CMAKE_SOURCE_DIR}/../image_converter
        NO_DEFAULT_PATH
        REQUIRED)

file(GLOB_RECURSE IMAGE_FILES "Images/*.*")
list(FILTER IMAGE_FILES EXCLUDE REGEX "\\.(txt|cmake)$")
set(main_h_file "${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/main.h")

set(IMAGE_HEADERS)
foreach(file ${IMAGE_FILES})
    get_filename_component(name "${file}" NAME_WE)
    set(h_file "${CMAKE_CURRENT_BINARY_DIR}/img_${name}.h")
    set(png_file "${CMAKE_CURRENT_BINARY_DIR}/img_${name}.png")
    message(STATUS "Found image ${name}")
    add_custom_command(
            COMMAND "${Python3_EXECUTABLE}" "${IMAGE_CONVERTER_PY}" -x 400 -y 57 --double-y --png "${file}" "${h_file}"
            OUTPUT "${h_file}"
            BYPRODUCTS "${png_file}"
            MAIN_DEPENDENCY "${file}"
            DEPENDS "${main_h_file}"
    )
    list(APPEND IMAGE_HEADERS "${h_file}")
endforeach()
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

