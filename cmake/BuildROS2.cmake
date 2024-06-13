################################################
##          Find macros and libraries         ##
################################################

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(ontologenius REQUIRED)
find_package(ament_cmake_python REQUIRED)

###################################
##  ROS specific configuration   ##
###################################

set(TMP_INTERFACES "mementar")

macro(meme_queue_messages_generation)
    foreach (MSG ${ARGN})
        list(APPEND TMP_INTERFACES "msg/${MSG}")
    endforeach ()
endmacro(meme_queue_messages_generation)

macro(meme_queue_services_generation)
    foreach (SRV ${ARGN})
        list(APPEND TMP_INTERFACES "srv/${SRV}")
    endforeach()
endmacro(meme_queue_services_generation)

macro(meme_generate_interfaces)
    list(APPEND TMP_INTERFACES DEPENDENCIES builtin_interfaces std_msgs ontologenius)

    rosidl_generate_interfaces(${TMP_INTERFACES})

    ament_export_dependencies(rosidl_default_runtime pluginlib)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
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
    ament_target_dependencies(${TARGET}
        PUBLIC
            rclcpp
            pluginlib
            builtin_interfaces
            std_msgs
            ontologenius)

    target_link_libraries(${TARGET} PUBLIC
            ontologenius::ontologenius_lib
            # todo: I feel like I shouldn't be doing this ^
            ${cpp_typesupport_target})

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

    ament_export_libraries(${TARGET})
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
    install(
        TARGETS ${ARGN}
        EXPORT  mementar
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        # INCLUDES DESTINATION include
        DESTINATION lib/${PROJECT_NAME})
endfunction(meme_install_libs)

function(meme_install_executables)
    install(
        TARGETS ${ARGN}
        DESTINATION lib/${PROJECT_NAME})
endfunction(meme_install_executables)

include(cmake/BuildCommon.cmake)

install(DIRECTORY launch/ros2/launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY files DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY configuration DESTINATION share/${PROJECT_NAME}/)

# ament_export_targets(mementar)
ament_package()