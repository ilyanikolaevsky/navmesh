add_library(${PROJECT_NAME}
  point.cpp
  segment.cpp
  polygon.cpp
  path_finder.cpp
  cone_of_vision.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ../include/)
