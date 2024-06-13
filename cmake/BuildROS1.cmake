################################################
##          Find macros and libraries         ##
################################################

find_package(catkin REQUIRED COMPONENTS
        roscpp
        # rospy
        std_msgs
        genmsg
        message_generation
        pluginlib
        ontologenius
)

find_package(cmake_modules REQUIRED)
find_package(pluginlib REQUIRED)

###################################
##  ROS specific configuration   ##
###################################

macro(meme_queue_messages_generation)
    add_message_files(FILES ${ARGN})
endmacro(meme_queue_messages_generation)

macro(meme_queue_services_generation)
    add_service_files(FILES ${ARGN})
endmacro(meme_queue_services_generation)

macro(meme_generate_interfaces)
    generate_messages(
            DEPENDENCIES
            std_msgs
            ontologenius
    )
endmacro(meme_generate_interfaces)

###################################
##             Build             ##
###################################

function(meme_add_generic TARGET)
    set_target_properties(${TARGET}
            PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON)

    target_compile_options(${TARGET}
            PRIVATE
            -Wall -Wextra)

    target_enable_sanitizers(${TARGET})
endfunction(meme_add_generic)

function(meme_add_ros_generic TARGET)
    target_link_libraries(${TARGET} PUBLIC ${catkin_LIBRARIES})
    target_include_directories(${TARGET} PUBLIC ${catkin_INCLUDE_DIRS})
    add_dependencies(${TARGET} ${catkin_EXPORTED_TARGETS} mementar_gencpp)

    target_compile_definitions(${TARGET} PUBLIC MEME_ROS_VERSION=$ENV{ROS_VERSION})
    target_compile_definitions(${TARGET} PUBLIC ONTO_ROS_VERSION=$ENV{ROS_VERSION})
    meme_add_generic(${TARGET})
endfunction(meme_add_ros_generic)

function(meme_add_library TARGET)
    if(NOT TARGET)
        message(FATAL_ERROR "Expected the target name as first argument")
    endif()

    if(NOT ARGN)
        message(FATAL_ERROR "Expected source file list after target name")
    endif()

    add_library(${TARGET} ${ARGN})

    target_include_directories(${TARGET}
            PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)

    meme_add_generic(${TARGET})
endfunction(meme_add_library)

function(meme_add_ros_library TARGET)
    if(NOT TARGET)
        message(FATAL_ERROR "Expected the target name as first argument")
    endif()

    if(NOT ARGN)
        message(FATAL_ERROR "Expected source file list after target name")
    endif()

    add_library(${TARGET} ${ARGN})

    target_include_directories(${TARGET}
            PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)

    # ament_export_libraries(${TARGET})
    meme_add_ros_generic(${TARGET})
endfunction(meme_add_ros_library)

function(meme_add_ros_executable TARGET)
    if(NOT TARGET)
        message(FATAL_ERROR "Expected the target name as first argument")
    endif()

    if(NOT ARGN)
        message(FATAL_ERROR "Expected source file list after target name")
    endif()

    add_executable(${TARGET} ${ARGN})
    target_include_directories(${TARGET} PUBLIC include)
    meme_add_ros_generic(${TARGET})
endfunction(meme_add_ros_executable)

function(meme_install_libs)
    install(TARGETS ${ARGN}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endfunction(meme_install_libs)

function(meme_install_executables)
    install(TARGETS ${ARGN}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endfunction(meme_install_executables)


include(cmake/BuildCommon.cmake)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES mementar_lib
        CATKIN_DEPENDS roscpp std_msgs ontologenius
        DEPENDS OpenCV
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)