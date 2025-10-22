project(xCoreSDK LANGUAGES CXX)

# 定义导入的共享库
add_library(${PROJECT_NAME}_lib SHARED IMPORTED GLOBAL)
# 定义导入的静态库
add_library(${PROJECT_NAME}_static STATIC IMPORTED GLOBAL)
# 定义导入的xMateModel静态库
add_library(xmatemodel_lib STATIC IMPORTED) # 这里的GLOBAL可以根据需要移除

if(UNIX)
    if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
        if(XCORE_USE_XMATE_MODEL)
            set_target_properties(${PROJECT_NAME}_lib PROPERTIES
                INTERFACE_LINK_LIBRARIES xmatemodel_lib)
        else()
            set(SHARED_LIB_SUFFIX ".nomodel")
        endif()
    elseif((NOT ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64") OR ${XCORE_USE_XMATE_MODEL})
        message(FATAL_ERROR "Unsupported: target ${CMAKE_SYSTEM_PROCESSOR} OR XCORE_USE_XMATE_MODEL option")
    endif()

    # set path to model library
    set_target_properties(xmatemodel_lib PROPERTIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/Linux/cpp/x86_64/libxMateModel.a)

    set(XCORESDK_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Linux/cpp/${CMAKE_SYSTEM_PROCESSOR})
    set_target_properties(${PROJECT_NAME}_lib PROPERTIES
        IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/lib${PROJECT_NAME}${SHARED_LIB_SUFFIX}.so.${CMAKE_PROJECT_VERSION}
        )
    set_target_properties(${PROJECT_NAME}_static PROPERTIES
        IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/lib${PROJECT_NAME}.a
        INTERFACE_LINK_LIBRARIES xmatemodel_lib
        )

else()
    message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}. Only Linux is explicitly configured.")
endif()

if(XCORE_LINK_SHARED_LIBS)
    add_library(Rokae::Rokae ALIAS ${PROJECT_NAME}_lib)
else()
    add_library(Rokae::Rokae ALIAS ${PROJECT_NAME}_static)
endif()

# --- 安装配置（可选，但推荐）---
# 在安装时，确保共享库被放置到目标系统的可执行文件目录
# 注意：IMPORTED_RUNTIME_ARTIFACTS 仅适用于共享库
#install(IMPORTED_RUNTIME_ARTIFACTS ${PROJECT_NAME}_lib
#    RUNTIME
#    DESTINATION ${CMAKE_INSTALL_LIBDIR}
#)