# 获取当前目录下的所有子目录
file(GLOB children RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(child ${children})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${child})
        # 检查子目录中是否存在CMakeLists.txt
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${child}/CMakeLists.txt)
            message(STATUS "Found CMakeLists.txt in subdirectory: ${child}")
            add_subdirectory(${child})
        endif()
    endif()
endforeach()