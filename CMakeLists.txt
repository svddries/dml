cmake_minimum_required(VERSION 2.8.3)
project(dml)

find_package(catkin REQUIRED COMPONENTS
    rgbd
    armadillo_matrix
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(dml
    src/mesh_tools.cpp       include/dml/mesh_tools.h
    include/dml/line_estimator.h
)

target_link_libraries(dml ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# ------------------------------------------------------------------------------------------------

add_executable(dml_test test/dml_test.cpp)
target_link_libraries(dml_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(dml_test_icp test/dml_test_icp.cpp)
target_link_libraries(dml_test_icp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(dml_test_poly test/dml_test_poly.cpp)
target_link_libraries(dml_test_poly ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(dml_test_fitter test/dml_test_fitter.cpp)
target_link_libraries(dml_test_fitter dml ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(dml_test_ground_filter test/dml_test_ground_filter.cpp)
target_link_libraries(dml_test_ground_filter dml ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(dml_test_line_estimator test/test_line_estimator.cpp)
target_link_libraries(dml_test_line_estimator dml ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(dml_test_ground_filter2 test/dml_test_ground_filter2.cpp)
target_link_libraries(dml_test_ground_filter2 dml ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
