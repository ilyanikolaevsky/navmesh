project(navmesh)

cmake_minimum_required(VERSION 3.19)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# To improve performance: disable exceptions, enable optimisations, etc.
# Copied these params from the original Visual Studio project, which resulted in a twice faster demo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /GL /Gy /Oi /Ot /O3 /permissive-")


# gtest sets "/MT" flag for static linking with the runtime library.
# For tests project to link, it has to use the same mode.
foreach(flag_var
    CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
    CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
  if(${flag_var} MATCHES "/MD")
    string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
  endif()
  if (${flag_var} MATCHES "/GR")
   string(REGEX REPLACE "/GR" "" ${flag_var} "${${flag_var}}")
  endif()
endforeach()

include_directories(source)

enable_testing()
add_subdirectory(third_party)
add_subdirectory(source)
add_subdirectory(tests)
if (CMAKE_GENERATOR MATCHES "Visual Studio")
  add_subdirectory(demo)
endif()