cmake_minimum_required(VERSION 2.8.3)

macro(use_cxx11)
    if (CMAKE_VERSION VERSION_LESS "3.1")
        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
        endif ()
    else ()
        set(CMAKE_CXX_STANDARD 11)
    endif ()
endmacro(use_cxx11)

project(multi_uav_simulator)

find_package(Armadillo REQUIRED)
find_package(GSL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        #rospy
        std_msgs
        geometry_msgs
        tf
        geo_controller
        message_generation
        simulator_utils
        mav_trajectory_generation
        )

catkin_package(
)

include_directories(
        include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
        ${geo_controller_INCLUDE_DIRS}
        ${simulator_utils_INCLUDE_DIRS}
)

add_executable(multi_uav_simulator
        src/DynamicsProvider.cpp
        src/cfSimUtils.cpp
        src/Quadrotor.cpp
        )

add_executable(send_trajectory
        src/send_trajectory.cpp
        )

add_executable(send_trajectory_robot2
        src/send_trajectory_robot2.cpp
        )

add_executable(send_trajectory_robot3
        src/send_trajectory_robot3.cpp
        )

add_executable(send_trajectory_robot4
        src/send_trajectory_robot4.cpp
        )
add_executable(send_trajectory_explore
        src/send_trajectory_explore.cpp
        )        


target_link_libraries(send_trajectory
        GSL::gsl
        GSL::gslcblas
        ${catkin_LIBRARIES})

target_link_libraries(send_trajectory_robot2
        GSL::gsl
        GSL::gslcblas
        ${catkin_LIBRARIES})

target_link_libraries(send_trajectory_robot3
        GSL::gsl
        GSL::gslcblas
        ${catkin_LIBRARIES})

target_link_libraries(send_trajectory_explore
        GSL::gsl
        GSL::gslcblas
        ${catkin_LIBRARIES})        

target_link_libraries(send_trajectory_robot4
        GSL::gsl
        GSL::gslcblas
        ${catkin_LIBRARIES})

target_link_libraries(multi_uav_simulator
        GSL::gsl
        GSL::gslcblas
        ${catkin_LIBRARIES})
