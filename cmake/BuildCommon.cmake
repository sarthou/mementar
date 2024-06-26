# This file should only be included by ROS1.cmake or ROS2.cmake which define the following functions:
# - meme_add_library
# - meme_add_ros_library (target linked with ROS-related deps)
# - meme_add_ros_executable (target linked with ROS-related deps)

include(cmake/Sanitizers.cmake)

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################
meme_queue_messages_generation(
    MementarAction.msg
    MementarExplanation.msg
    MementarOccasion.msg
    MementarTimestamp.msg
    StampedFact.msg
    StampedString.msg)

meme_queue_services_generation(
    MementarOccasionSubscription.srv
    MementarOccasionUnsubscription.srv
    MementarService.srv)

meme_generate_interfaces()

# ##################################
# #      Compatibility layer      ##
# ##################################
meme_add_ros_library(mementar_compat
    src/compat/ros.cpp)

# ##################################
meme_add_ros_library(mementar_events_lib
    src/core/Occasions/Subscription.cpp
    src/core/Occasions/OccasionsManager.cpp)

# ##################################
meme_add_ros_library(mementar_compression_lib
    src/core/LtManagement/archiving_compressing/binaryManagement/BitFileGenerator.cpp
    src/core/LtManagement/archiving_compressing/binaryManagement/BitFileGetter.cpp
    src/core/LtManagement/archiving_compressing/compressing/LzCompress.cpp
    src/core/LtManagement/archiving_compressing/compressing/LzUncompress.cpp
    src/core/LtManagement/archiving_compressing/compressing/Huffman.cpp
    src/core/LtManagement/archiving_compressing/archiving/Header.cpp
    src/core/LtManagement/archiving_compressing/archiving/Archive.cpp)

meme_add_ros_library(mementar_memGraphs_lib
    src/core/memGraphs/Branchs/ValuedNode.cpp
    src/core/memGraphs/Branchs/types/Action.cpp
    src/core/memGraphs/Branchs/types/Fact.cpp
    src/core/memGraphs/Branchs/types/SoftPoint.cpp
    src/core/memGraphs/Branchs/types/Triplet.cpp
    src/core/memGraphs/Graphs/ActionGraph.cpp
    src/core/memGraphs/Graphs/FactGraph.cpp)

meme_add_ros_library(mementar_lt_lib
    src/core/LtManagement/EpisodicTree/CompressedLeaf.cpp
    src/core/LtManagement/EpisodicTree/CompressedLeafNode.cpp
    src/core/LtManagement/EpisodicTree/Context.cpp
    src/core/LtManagement/EpisodicTree/CompressedLeafSession.cpp
    src/core/LtManagement/EpisodicTree/CompressedLeafNodeSession.cpp
    src/core/LtManagement/EpisodicTree/ArchivedLeaf.cpp
    src/core/LtManagement/EpisodicTree/ArchivedLeafNode.cpp)

target_link_libraries(mementar_lt_lib
    PUBLIC
    mementar_compression_lib
    mementar_memGraphs_lib
    pthread)

meme_add_ros_library(mementar_core_lib
    src/core/feeder/FeedStorage.cpp
    src/core/feeder/Feeder.cpp
    src/core/Parametrization/Configuration.cpp)

target_link_libraries(mementar_core_lib
    PUBLIC
    mementar_lt_lib)

# ##################################
meme_add_ros_library(mementar_drawer_lib
    src/graphical/timeline/ActionReader.cpp
    src/graphical/timeline/FactReader.cpp
    src/graphical/timeline/TimelineDrawer.cpp)

target_include_directories(mementar_drawer_lib
    PUBLIC
    ${OpenCV_INCLUDE_DIRS})

target_link_libraries(mementar_drawer_lib
    PUBLIC
    ${OpenCV_LIBS}
    mementar_memGraphs_lib)

# ##################################
meme_add_ros_library(mementar_lib
    src/API/mementar/ActionsPublisher.cpp
    src/API/mementar/ActionsSubscriber.cpp
    src/API/mementar/TimelineManipulator.cpp
    src/API/mementar/TimelinesManipulator.cpp
    src/API/mementar/clients/ActionClient.cpp
    src/API/mementar/clients/ClientBase.cpp
    src/API/mementar/clients/FactClient.cpp
    src/API/mementar/clients/ManagerClient.cpp
    src/API/mementar/clients/InstanceManagerClient.cpp
    src/API/mementar/OccasionsPublisher.cpp
    src/API/mementar/OccasionsSubscriber.cpp)
target_include_directories(mementar_lib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mementar/API>
    $<INSTALL_INTERFACE:include/mementar/API>)
target_link_libraries(mementar_lib PUBLIC mementar_compat
)

# ##################################
meme_add_ros_library(mementar_interface
    src/RosInterface.cpp
    src/graphical/timeline/CsvSaver.cpp)

target_link_libraries(mementar_interface
    PUBLIC
    mementar_core_lib
    mementar_events_lib
    mementar_drawer_lib)

# ##################################
meme_add_ros_executable(mementar_single src/nodes/mementar_single.cpp)
target_link_libraries(mementar_single PUBLIC mementar_interface mementar_compat)

meme_add_ros_executable(mementar_multi src/nodes/mementar_multi.cpp)
target_link_libraries(mementar_multi PUBLIC mementar_interface mementar_compat)

meme_add_ros_executable(mementar_timeline src/graphical/timeline/main.cpp)
target_link_libraries(mementar_timeline PUBLIC mementar_core_lib mementar_drawer_lib)

# ##################################

# #############################################################################
# Qt Environment
# #############################################################################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# #############################################################################
# Sections
# #############################################################################
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

# #############################################################################
# Sources
# #############################################################################
set(QT_SOURCES
    src/graphical/mementarGUI/main.cpp
    src/graphical/mementarGUI/mementargui.cpp
    src/graphical/mementarGUI/DarkStyle.cpp
    src/graphical/mementarGUI/QPushButtonExtended.cpp
    src/graphical/mementarGUI/QCheckBoxExtended.cpp
    src/graphical/mementarGUI/QLineEditExtended.cpp
    include/mementar/graphical/mementarGUI/mementargui.h
    include/mementar/graphical/mementarGUI/DarkStyle.h
    include/mementar/graphical/mementarGUI/QPushButtonExtended.h
    include/mementar/graphical/mementarGUI/QCheckBoxExtended.h
    include/mementar/graphical/mementarGUI/QLineEditExtended.h)

# #############################################################################
# Binaries
# #############################################################################
add_executable(mementarGUI ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
set_target_properties(mementarGUI PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(mementarGUI PUBLIC -DMEME_ROS_VERSION=$ENV{ROS_VERSION})
target_link_libraries(mementarGUI
    mementar_lib
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport)
target_include_directories(mementarGUI PUBLIC include)

meme_install_executables(
    mementar_single
    mementar_multi
    mementar_timeline
    mementarGUI)

# ##################################