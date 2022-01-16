# Neglect
cmake_minimum_required(VERSION 3.8)


# [1]
########  Replace tracking with your project name ########
project(tracking)
##########################################################





# Neglect
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




# [2]
#################### find_package list #################### 
find_package(ament_cmake REQUIRED)
############## Put your chosen packages here  #############
find_package(rclcpp REQUIRED)          # (i)
find_package(std_msgs REQUIRED)        # (ii)
find_package(sensor_msgs REQUIRED)     # (iii)
find_package(rtsd_interfaces REQUIRED) # (iv)
###########################################################




# [3]
################### Create executables ###################
add_executable(tracking src/cpp_tracking.cpp) #(executable1)
ament_target_dependencies(
  tracking         # (executable1)
  rclcpp           # (i)
  std_msgs         # (ii)
  sensor_msgs      # (iii)
  rtsd_interfaces) # (iv)
##########################################################





# [4]
########### Install the executables somewhere  ###########
install(TARGETS
tracking  #(executable1)
DESTINATION lib/${PROJECT_NAME})
##########################################################






# Neglect
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()
ament_package()