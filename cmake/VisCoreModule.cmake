#[[
# =============================================================================
# 函数名: VisCore_add_module
#
# 功能: 为 VisCore 项目添加静态库/接口库模块
#
# 典型调用示例:
#   # 添加静态库模块
#   VisCore_add_module(armor_detector
#       DEPENDS utils geometry
#       EXTRA_HEADER ${OpenCV_INCLUDE_DIRS}
#       EXTERNAL ${OpenCV_LIBS}
#   )
#
#   # 添加接口库模块
#   VisCore_add_module(trajectory_planner
#       INTERFACE
#       DEPENDS math_utils
#   )
#
# 参数说明:
#   module_name - 模块基础名称(会自动添加"VisCore_"前缀)
#
#   [可选] INTERFACE - 指定则创建纯头文件接口库
#
#   [可选] DEPENDS - VisCore模块列表(无需前缀)
#           示例: DEPENDS module1 module2
#
#   [可选] EXTRA_HEADER - 第三方库头文件搜索路径
#           示例: EXTRA_HEADER /path/to/include
#
#   [可选] EXTERNAL - 需要链接的第三方库目标
#           示例: EXTERNAL lib1 lib2
#
# 自动行为:
#   1. 库目标名称: VisCore_${module_name}
#   2. 源文件搜索路径(按顺序尝试):
#      - ${CMAKE_CURRENT_LIST_DIR}/src/${module_name}/
#      - ${CMAKE_CURRENT_LIST_DIR}/src/
#   3. 头文件自动包含路径:
#      - ${CMAKE_CURRENT_LIST_DIR}/include/
#
# 副作用:
#   更新以下全局缓存变量:
#   - VisCore_MODULES_PUBLIC    (记录所有静态/动态库)
#   - VisCore_MODULES_INTERFACE (记录所有接口库)
#   - VisCore_MODULES_BUILD     (记录所有构建的库)
# =============================================================================
]]
function(VisCore_add_module module_name)
    # 定义三类参数：
    set(options INTERFACE)                  # 标志参数（无值，表示布尔开关）
    set(oneValueArgs "")                    # 单值参数（此示例未使用）
    set(multiValueArgs DEPENDS EXTRA_HEADER EXTERNAL)  # 多值参数

    cmake_parse_arguments(MD "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # 前缀名
    set(module_prefix "VisCore_")
    # 创建模块全名
    set(module_lib_name "${module_prefix}${module_name}")

    # 创建模块的库
    if(MD_INTERFACE)
        add_library(${module_lib_name} INTERFACE)
    else()
        add_library(${module_lib_name})
    endif()

    # 模块添加源
    if (NOT MD_INTERFACE)
        # 寻找源文件目录
        set(module_source_dir ${CMAKE_CURRENT_LIST_DIR}/src/${module_name})
        if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/src/${module_name})
            set(module_source_dir ${CMAKE_CURRENT_LIST_DIR}/src)
        endif()
        # 递归获取所有 .c 和 .cpp 文件
        file(GLOB_RECURSE module_sources_src
            "${module_source_dir}/*.c"
            "${module_source_dir}/*.cpp"
        )
        target_sources(${module_lib_name} PRIVATE ${module_sources_src})
    endif()

    # 添加依赖
    if(MD_INTERFACE)
        # 添加头文件和第三方库依赖头文件
        target_include_directories(${module_lib_name} INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/include
            ${MD_EXTRA_HEADER})
        # VisCore公共模块依赖头文件
        foreach(depends_modules ${MD_DEPENDS})
            set(depends_module_lib_name "${module_prefix}${depends_modules}")
            target_link_libraries(${module_lib_name} INTERFACE ${depends_module_lib_name})
        endforeach()
        # 添加第三方库依赖
        foreach(external_lib ${MD_EXTERNAL})
            target_link_libraries(${module_lib_name} INTERFACE ${external_lib})
        endforeach()
    else() # 动态库或静态库链接
        # 添加头文件和第三方库依赖头文件
        target_include_directories(${module_lib_name} PUBLIC
            ${CMAKE_CURRENT_LIST_DIR}/include
            ${MD_EXTRA_HEADER})
        # VisCore公共模块依赖头文件
        foreach(depends_modules ${MD_DEPENDS})
            set(depends_module_lib_name "${module_prefix}${depends_modules}")
            target_link_libraries(${module_lib_name} PUBLIC ${depends_module_lib_name})
        endforeach()
        # 添加第三方库依赖
        foreach(external_lib ${MD_EXTERNAL})
            target_link_libraries(${module_lib_name} PUBLIC ${external_lib})
        endforeach()
    endif()

    # 记录构建的模块 
    if(NOT MD_INTERFACE)
        set(VisCore_MODULES_PUBLIC ${VisCore_MODULES_PUBLIC} ${module_lib_name} CACHE INTERNAL "VisCore公共模块" FORCE)
    else()
        set(VisCore_MODULES_INTERFACE ${VisCore_MODULES_INTERFACE} ${module_lib_name} CACHE INTERNAL "VisCore接口模块" FORCE)
    endif()
    set(VisCore_MODULES_BUILD ${VisCore_MODULES_BUILD} ${module_lib_name} CACHE INTERNAL "VisCore构建库" FORCE)
endfunction()

        
function(VisCore_add_exe exe_name)
    # 处理函数参数
    set(multi_args EXTRA_HEADER DEPENDS EXTERNAL)
    cmake_parse_arguments(EXE "" "" "${multi_args}" ${ARGN})

    # 添加可执行文件的头文件
    set(exe_include_dir ${CMAKE_CURRENT_LIST_DIR}/include)

    # 获取 src 目录下的所有源文件
    set(exe_source_dir ${CMAKE_CURRENT_LIST_DIR}/src)
    set(exe_sources_src "")
    aux_source_directory(${exe_source_dir} exe_sources_src)

    set(exe_prefix "VisCore_")
    set(exe_suffix "_exe")
    set(target_name "${exe_prefix}${exe_name}${exe_suffix}")

    # main.cpp 路径
    set(main_cpp_path ${CMAKE_CURRENT_LIST_DIR}/main.cpp)

    # 添加可执行文件
    add_executable(${target_name} ${main_cpp_path} ${exe_sources_src})
    # 设置可执行文件的头文件搜索路径
    target_include_directories(${target_name} PUBLIC ${exe_include_dir} ${EXE_EXTRA_HEADER})

    # 添加依赖
    foreach(depends_module ${EXE_DEPENDS})
        target_link_libraries(${target_name} PUBLIC VisCore_${depends_module})
    endforeach()
    target_link_libraries(${target_name} PUBLIC ${EXE_EXTERNAL})
endfunction()
