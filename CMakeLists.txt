cmake_minimum_required(VERSION 3.14)
project(navmesh)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# To improve performance: disable exceptions, enable optimisations, etc.
# Copied these params from the original Visual Studio project, which resulted in a twice faster demo
if (CMAKE_GENERATOR MATCHES "Visual Studio")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /GL /Gy /Oi /Ot /O3 /permissive-")
endif()


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

add_subdirectory(src)

include(CTest)
if (BUILD_TESTING)
  add_subdirectory(test)
endif()

if (CMAKE_GENERATOR MATCHES "Visual Studio")
  add_subdirectory(demo)
endif()
