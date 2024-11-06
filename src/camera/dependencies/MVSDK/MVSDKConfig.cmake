# # ament_export_targets(MVSDK HAS_LIBRARY_TARGET)
# # # ament_export_dependencies(some_dependency)

# # file(GLOB_RECURSE include_files "${CMAKE_CURRENT_SOURCE_DIR}/include/*")
# # add_library(MVSDK include_files)

# # install(
# #   DIRECTORY include/
# #   DESTINATION include
# # )

# # install(
# #   TARGETS MVSDK
# #   EXPORT MVSDK
# #   LIBRARY DESTINATION lib
# #   ARCHIVE DESTINATION lib
# #   INCLUDES DESTINATION include
# # )

# target_include_directories(MVSDK
#   PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")

# install(
#   DIRECTORY include/
#   DESTINATION include
# )
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

# ament_export_include_directories("${MVSDK_DIR}/include")
# ament_export_libraries(MVSDK)

# ament_package()
