cmake_minimum_required(VERSION 3.1)
project(display_image)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find external modules
find_package(OpenCV 3 REQUIRED)

# Compile options
if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") OR (CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wcast-qual -Wcast-align)
endif()

# Create a list with all .cpp source files
set( projectLib
   #src/FeaturesDetection.cpp
)

#add_library (my_library ${projectLib})

# Target executable
#add_executable(mainT_seq3 src/test_marker3.cpp)
add_executable(mainT_seq1 src/test_marker1.cpp)
target_link_libraries(mainT_seq1 ${OpenCV_LIBS})
#target_link_libraries(my_library ${OpenCV_LIBS})
#target_link_libraries(mainT_seq3 my_library)
