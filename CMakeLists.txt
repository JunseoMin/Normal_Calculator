cmake_minimum_required(VERSION 3.10)

project(NormalCalculator VERSION 0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${Eigen3_INCLUDE_DIRS}
)

set(normalCalculator_src
  src/NormalCalculator.cpp
  src/mathtools.cpp
  src/DataHandler.cpp
)


add_executable(main_test src/main.cpp ${normalCalculator_src})

target_link_libraries(main_test
  PUBLIC
  Eigen3::Eigen
)
