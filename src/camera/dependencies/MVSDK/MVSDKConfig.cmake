# 设置包含路径和库路径
set(MVSDK_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")
set(MVSDK_LIB "${CMAKE_CURRENT_LIST_DIR}/lib/libMVSDK.so")

# 定义导出的库目标
add_library(MVSDK SHARED IMPORTED)
set_target_properties(MVSDK PROPERTIES
    IMPORTED_LOCATION "${MVSDK_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${MVSDK_INCLUDE_DIR}"
)

# 设置给 find_package 的输出变量
set(MVSDK_FOUND TRUE)
