cmake_minimum_required(VERSION 3.4)
project(factor_graph_node)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++17)

# This line is required for specifying the "modules" directory in the project containing .cmake files
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/../../devel;/opt/ros/noetic")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(GTSAM REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(Vendor/Geometry/)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${GTSAM_INCLUDE_DIR})

message(STATUS "Geometry:${Geometry_INCLUDE_DIR}")

include_directories(      # include
        ${catkin_INCLUDE_DIRS}
        ${Geometry_INCLUDE_DIR}
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  map_sense
  roscpp
  rospy
  std_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs map_sense
)

set(CLAY_INCLUDE_DIRS
        "/usr/local/include/ClayEngine"
        "/usr/local/include/ClayEngine/Vendor/"
        "/usr/local/include/ClayEngine/Vendor/GLAD/include"
        "/usr/local/include/ClayEngine/Vendor/glfw/include"
        "/usr/local/include/ClayEngine/Vendor/glm"
        "/usr/local/include/ClayEngine/Vendor/ImGui"
        "/usr/local/include/ClayEngine/Vendor/ImGui/imgui"
        "/usr/local/include/ClayEngine/Vendor/spdlog/include"
        "/usr/local/include/ClayEngine/Vendor/stb_image/")

include_directories(
        include
        src/headers
        src/Geometry/include
        ${catkin_INCLUDE_DIRS}
        /usr/include/CL/
        Vendor/spdlog/include
        Vendor/xtensor/include
        Vendor/xtensor-blas/include
        /usr/local/include/ClayEngine/Vendor/
        Vendor/implot/
        ${CLAY_INCLUDE_DIRS}
)

set(FG_SOURCES


        src/ApplicationState.cpp
        src/FactorGraphHandler.cpp
        src/NetworkManager.cpp

        include/NetworkManager.h
        include/ApplicationState.h
        include/FactorGraphHandler.h
        include/Log.h

        src/SLAM.cpp include/SLAM.h)


add_library(implot STATIC Vendor/implot/implot.h Vendor/implot/implot_internal.h Vendor/implot/implot.cpp Vendor/implot/implot_items.cpp Vendor/implot/implot_demo.cpp)
target_include_directories(implot PUBLIC Vendor/implot/
        "/usr/local/include/ClayEngine/Vendor/ImGui"
        "/usr/local/include/ClayEngine/Vendor/ImGui/imgui")

set(FACTOR_GRAPH_NODE 0)
if (FACTOR_GRAPH_NODE)
    add_executable(FactorGraphNode ${FG_SOURCES}
            src/FactorGraphNodeApp.cpp
            src/MeshGenerator.cpp
            include/MeshGenerator.h
            src/ApplicationLayer.cpp
            include/ApplicationLayer.h
            src/FactorGraphNodeLayer.cpp
            include/FactorGraphNodeLayer.h include/Log.h)
    target_link_libraries(FactorGraphNode
            ${catkin_LIBRARIES} gtsam geom-tools
            ClayEngine glad imgui implot glfw3 ${CMAKE_DL_LIBS} ${BLAS_LIBRARIES})
    add_dependencies(FactorGraphNode map_sense_generate_messages_cpp)
    target_include_directories(FactorGraphNode PUBLIC
            ${Geometry_INCLUDE_DIR}
            ${catkin_INCLUDE_DIRS}
            ${CLAY_INCLUDE_DIRS}
            /usr/local/include/ClayEngine/Vendor/
            Vendor/implot/
            include
            )
    target_compile_definitions(FactorGraphNode PUBLIC -DASSETS_PATH="/usr/local/include/ClayEngine/Assets/")
endif ()

set(FACTOR_GRAPH_NODE_HEADLESS 1)
if (FACTOR_GRAPH_NODE_HEADLESS)
    add_executable(FactorGraphNodeHeadless ${FG_SOURCES}  src/FactorGraphNodeHeadless.cpp include/FactorGraphNodeHeadless.h)
    target_link_libraries(FactorGraphNodeHeadless
            ${catkin_LIBRARIES} gtsam geom-tools ${CMAKE_DL_LIBS} ${BLAS_LIBRARIES})
    add_dependencies(FactorGraphNodeHeadless map_sense_generate_messages_cpp)
    target_include_directories(FactorGraphNodeHeadless PUBLIC
            ${Geometry_INCLUDE_DIR}
            ${catkin_INCLUDE_DIRS}
            include
            )
    target_compile_definitions(FactorGraphNodeHeadless PUBLIC -DASSETS_PATH="/usr/local/include/ClayEngine/Assets/")
    if(LOG_DEBUG)
        target_compile_definitions(FactorGraphNodeHeadless PUBLIC -DLOG_DEBUG=1)
    endif()
endif ()

