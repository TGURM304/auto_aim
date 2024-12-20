# 设置包含路径和库路径
set(HKSDK_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")
set(HKSDK_LIB "${CMAKE_CURRENT_LIST_DIR}/lib/libMVCameraControl.so")

# 定义导出的库目标
add_library(HKSDK SHARED IMPORTED)
set_target_properties(HKSDK PROPERTIES
    IMPORTED_LOCATION "${HKSDK_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${HKSDK_INCLUDE_DIR}"
)

# 设置给 find_package 的输出变量
set(HKSDK_FOUND TRUE)
