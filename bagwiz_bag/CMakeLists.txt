cmake_minimum_required(VERSION 3.16)
project(bagwiz_bag CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Scope to C++ only. When BAGWIZ_WITH_SLAM_CUDA enables the CUDA language,
  # these flags would otherwise reach nvcc, which forwards -Wpedantic to the
  # host compiler on its generated intermediate (.cudafe1.stub.c). That file is
  # full of GCC-style linemarkers, so -Wpedantic floods the build with
  # "style of line directive is a GCC extension" — one per line of the .cu.
  add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-Wshadow>")
endif()

find_package(ament_cmake REQUIRED)
find_package(bagwiz_base REQUIRED)
find_package(bagwiz_io REQUIRED)
# Threads::Threads supplies -pthread for the std::thread in PipelinedBackend
# and the mutex/condition_variable in the public bounded_message_queue.hpp, so
# it is linked PUBLIC: every consumer of the pipeline headers needs it too.
find_package(Threads REQUIRED)

add_library(bagwiz_bag
  src/core/bag/bag_copy.cpp
  src/core/bag/bag_inplace.cpp
  src/core/bag/bag_topic_plan.cpp
  src/core/bag/broken_bag.cpp
  src/core/pipeline/backend_select.cpp
  src/core/pipeline/pipelined_backend.cpp
  src/core/pipeline/rewrite_backend.cpp
  src/core/pipeline/sequential_backend.cpp
  src/core/pipeline/stage_profiler.cpp
  src/core/pipeline/topic_router.cpp
)
target_include_directories(bagwiz_bag
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(bagwiz_bag PUBLIC bagwiz_base::bagwiz_base bagwiz_io::bagwiz_io Threads::Threads)

install(TARGETS bagwiz_bag
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(bagwiz_base bagwiz_io)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(bagwiz_bag_copy_test test/core/bag/bag_copy_test.cpp)
  target_link_libraries(bagwiz_bag_copy_test bagwiz_bag)

  ament_add_gtest(bagwiz_bag_inplace_test test/core/bag/bag_inplace_test.cpp)
  target_link_libraries(bagwiz_bag_inplace_test bagwiz_bag)

  ament_add_gtest(bagwiz_bag_topic_plan_test test/core/bag/bag_topic_plan_test.cpp)
  target_link_libraries(bagwiz_bag_topic_plan_test bagwiz_bag)

  ament_add_gtest(bagwiz_broken_bag_test test/core/bag/broken_bag_test.cpp)
  target_link_libraries(bagwiz_broken_bag_test bagwiz_bag)

  ament_add_gtest(bagwiz_stage_profiler_test test/core/pipeline/stage_profiler_test.cpp)
  target_link_libraries(bagwiz_stage_profiler_test bagwiz_bag)

  ament_add_gtest(bagwiz_rewrite_backend_test test/core/pipeline/rewrite_backend_test.cpp)
  target_link_libraries(bagwiz_rewrite_backend_test bagwiz_bag)

  ament_add_gtest(bagwiz_pipelined_backend_test test/core/pipeline/pipelined_backend_test.cpp)
  target_link_libraries(bagwiz_pipelined_backend_test bagwiz_bag)
endif()

ament_package()
