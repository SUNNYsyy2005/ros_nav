cmake_minimum_required(VERSION 3.16)
project(teb)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
find_package(Boost REQUIRED COMPONENTS system thread graph)
# find_package(SUITESPARSE REQUIRED) # 注释掉这一行
find_package(G2O REQUIRED) # 注释掉这一行
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES teb
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  /usr/include/eigen3
  /usr/include/suitesparse # 假设SuiteSparse的头文件在这个目录
  ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  message(STATUS "G2O_INCLUDE_DIRS: ${G2O_INCLUDE_DIRS}")
  ${catkin_INCLUDE_DIRS}
  #/usr/local/include/g2o # 假设G2O的头文件在这个目录
)

add_library(lib_teb
        src/teb_config.cpp
        src/obstacles.cpp
        src/visualization.cpp
        src/optimal_planner.cpp
        src/timed_elastic_band.cpp
)
add_executable(teb main.cpp)
target_link_libraries(
        teb
        lib_teb
        ${catkin_LIBRARIES}
        #g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension
        ${G2O_LIBRARIES}
        # /usr/lib/x86_64-linux-gnu/libumfpack.so 
        # /usr/lib/x86_64-linux-gnu/libcholmod.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_core.so 
        # /usr/local/lib/x86_64-linux-gnu/libg2o_stuff.so 
        # /usr/local/lib/x86_64-linux-gnu/libg2o_csparse_extension.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_ext_freeglut_minimal.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_hierarchical.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_incremental.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_interactive.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_interface.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_opengl_helper.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_parser.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_simulator.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_solver_cholmod.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_solver_dense.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_solver_csparse.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_solver_eigen.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_solver_pcg.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_solver_slam2d_linear.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_solver_structure_only.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_types_slam2d.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_types_slam3d.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_types_sclam2d.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_types_icp.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_types_sba.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_types_sim3.so
        # /usr/local/lib/x86_64-linux-gnu/libg2o_types_data.so
        Boost::system Boost::thread Boost::graph
        #-lboost_system -lboost_thread -lboost_graph
        ${OpenCV_LIBS}
)