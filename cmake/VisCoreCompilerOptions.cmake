# ----------------------------------------------------------------------------
#   编译选项
# ----------------------------------------------------------------------------
# 编译类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()


# 设置编译标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 强制遵守
set(CMAKE_CXX_EXTENSIONS OFF)# 禁止编译器拓展的 C++ 语法

# 设置默认库的编译类型 动态库 Shared 或 静态库 Static
option(BUILD_SHARED_LIBS "Build shared libraries(构建动态库)" ON)

# 设置编译优化选项
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG") # 优化级别 3
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -DDEBUG") # 不优化，开启调试信息
else()
    message(WARNING "Unknown build type: ${CMAKE_BUILD_TYPE}. Using default Release settings.(未知的编译类型: ${CMAKE_BUILD_TYPE}，使用默认的 Release 设置)")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
endif()


# 设置可执行文件的输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

# 生成 compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
