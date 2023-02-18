include(ExternalProject)
include(FetchContent)

## GoogleTest
FetchContent_Declare(
        googletest
        GIT_REPOSITORY  https://github.com/google/googletest.git
        GIT_TAG         v1.13.0
)
FetchContent_MakeAvailable(googletest)