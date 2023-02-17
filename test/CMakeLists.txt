include(ExternalProject)
include(FetchContent)

## GoogleTest
FetchContent_Declare(
        googletest
        GIT_REPOSITORY  https://github.com/google/googletest.git
        GIT_TAG         v1.13.0
)
FetchContent_MakeAvailable(googletest)

add_executable(${PROJECT_NAME}-test tests.cpp)
target_include_directories(${PROJECT_NAME}-test PRIVATE ../include)
target_link_libraries(${PROJECT_NAME}-test gtest gtest_main navmesh)
