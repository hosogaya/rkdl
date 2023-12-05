cmake_minimum_required(VERSION 3.0)

project(rkdl)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON) 

find_package(Eigen3 REQUIRED)

set(source_file
    src/joint/joint_base.cpp
    src/joint/fixed.cpp
    src/joint/revolute.cpp
    src/robot_model.cpp
    src/frame.cpp
    src/kinematics/solver.cpp
)

add_library(${PROJECT_NAME} SHARED ${source_file})

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    PUBLIC_HEADER
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    PUBLIC_HEADER DESTINATION include
)

install(EXPORT ${PROJECT_NAME}-export
    FILE ${PROJECT_NAME}-config.cmake
    DESTINATION share/cmake/${PROJECT_NAME}
    EXPORT_LINK_INTERFACE_LIBRARIES
)

function(add_rkdl_test base_name)
	add_executable(${base_name} test/${base_name}.cpp)
	target_link_libraries(${base_name} ${PROJECT_NAME})
    target_include_directories(${base_name}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    # install(
    #     TARGETS ${base_name}
    #     DESTINATION bin/${PROJECT_NAME}
    # )
endfunction()

add_rkdl_test(robot_model)
add_rkdl_test(full_robot_model)