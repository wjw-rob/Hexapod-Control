cmake_minimum_required(VERSION 3.0.2)
project(wjw)

# 设置C++编译器在Release模式下的编译标志为-Ofast
# -Ofast启用比-O3更多的优化，包括违反一些标准合规性的优化
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

# 为C++编译器设置通用的编译标志
# -O3表示最高级别的优化，生成运行速度最快的代码
# -Wall表示启用所有警告，这有助于在编译时发现潜在的问题
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

# 为C编译器设置通用的编译标志
# 同样使用-O3进行最高级别的优化，并启用所有警告
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")

# 使用C++11标准
add_compile_options(-std=c++11)

# 查找catkin包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  gazebo_msgs
  geometry_msgs
  sensor_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

# 声明catkin包
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wjw
#  CATKIN_DEPENDS roscpp rospy std_msgs gazebo_msgs geometry_msgs sensor_msgs
#   DEPENDS Eigen3 OsqpEigen
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
)

# 链接目录（如果需要的话）
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# 搜索源文件
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

# 添加可执行文件
# add_executable(main src/main.cpp)
#(wjw 3.10修改 为什么原来不添加源文件不会报错？)
add_executable(main 

  src/common/hexpodRobot.cpp
  src/common/unitreeLeg.cpp 
  src/common/LowPassFilter.cpp  #(wjw 3.19)

  src/control/ControlFrame.cpp
  src/control/neural_bezier_curve.cpp
  src/control/Estimator.cpp       #(wjw 3.19)
  src/control/BalanceCtrl.cpp     #(wjw 3.19)
  src/control/ConvexMpc.cpp       #(wjw 3.19)
  src/control/TerrianEsti.cpp      #(wjw 3.19)


  
  src/FSM/FSM.cpp
  src/FSM/FSMState.cpp 
  src/FSM/State_FixedStand.cpp
  src/FSM/State_Passive.cpp
  src/FSM/State_Position.cpp      #(wjw 3.19)
  src/FSM/State_Force_Pos.cpp

  src/Gait/WaveGenerator.cpp 
  # src/Gait/FeetEndCal.cpp         #(wjw 3.19)
  # src/Gait/GaitGenerator.cpp      #(wjw 3.19)
  src/Gait/GaitGenerator_P.cpp      #(wjw 3.21)
  src/Gait/SupportTrajectory.cpp  #(wjw 3.19)
  src/Gait/SupportFeetEndP.cpp     #(wjw 3.19)

  src/interface/IOROS.cpp
  src/interface/KeyBoard.cpp
  src/interface/CmdPanel.cpp 

  src/quadProgpp/Array.cc         #(wjw 3.19)
  src/quadProgpp/QuadProg++.cc    #(wjw 3.19)


  src/main.cpp 
  
)

# 链接库
target_link_libraries(main
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)

#确保程序包含调试信息,让 CMake 在编译时使用调试模式
set(CMAKE_BUILD_TYPE Debug)

