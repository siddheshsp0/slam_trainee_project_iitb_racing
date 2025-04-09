cmake_minimum_required(VERSION 3.8)
project(slam_project)

# CUDA
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# find dependencies
find_package(ament_cmake REQUIRED)



# Adding include directory
include_directories(
  include
)


include_directories(/usr/local/cuda-12.1/include/) # for cuda


# Adding package dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dv_msgs)




# For cuda


find_package(CUDA REQUIRED) # CUDA
# Add CUDA-specific compilation flags
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++17 -Xcompiler -fPIC")
# Include CUDA headers
include_directories(${CUDA_INCLUDE_DIRS})
# Manually add cuBLAS library path
link_directories(/usr/local/cuda-12.1/lib64)

# Define your CUDA source files
set(SOURCES
    src/utils.cu
)
# Compile CUDA code into a shared library
cuda_add_library(utils SHARED ${SOURCES})
# Link CUDA libraries
target_link_libraries(utils ${CUDA_LIBRARIES} cublas)

ament_export_libraries(utils)



# Cuda end----------------- #


# ---------- Added by Siddhesh --------- #



# -------------------- Custom Library --------------------- #
# Adding library
# add_library(utils SHARED src/new.cpp) # commented for cuda
add_library(data_assoc_trainee SHARED src/data_assoc_trainee.cpp)
# Export the include directories
target_include_directories(utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(data_assoc_trainee PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(data_assoc_trainee dv_msgs rclcpp)
target_link_libraries(data_assoc_trainee Eigen3::Eigen)


# Install the library and headers
install(TARGETS utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS data_assoc_trainee
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
# ------------------------------------------- #

# Adding Executables
add_executable(localization_node src/localization_node.cpp)
add_executable(localization_node_ekf src/localization_node_ekf.cpp)

# Link the node with my custom librarymath_node
target_include_directories(localization_node PUBLIC
  include
)
target_link_libraries(localization_node utils)
target_link_libraries(localization_node_ekf data_assoc_trainee utils)

# Adding dependencies of executables
ament_target_dependencies(localization_node rclcpp sensor_msgs visualization_msgs tf2 tf2_geometry_msgs eufs_msgs)
ament_target_dependencies(localization_node_ekf rclcpp sensor_msgs visualization_msgs tf2 tf2_geometry_msgs eufs_msgs dv_msgs)


# Install Targets section
install(TARGETS
  localization_node
  localization_node_ekf
  DESTINATION lib/${PROJECT_NAME}
)





# install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)




# -------------------- Added by Siddhesh End ------------------ #




if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
