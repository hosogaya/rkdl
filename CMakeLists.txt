cmake_minimum_required(VERSION 3.0)

project(rkdl)

find_package(Eigen3 REQUIRED)

set(source_file
    src/joint/joint_base.cpp
    src/joint/fixed.cpp
    src/joint/revolute.cpp
    src/robot_model.cpp
    src/frame.cpp
    src/kinematics/solver.cpp
)

add_library(${PROJECT_NAME} ${source_file})

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

# add_compile_definitions(PURE_EIGEN)


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