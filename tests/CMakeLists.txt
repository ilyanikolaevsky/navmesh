add_executable(tests 
               tests.cpp)

target_link_libraries(tests gtest gtest_main navmesh)

#target_include_directories(tests PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/gtest")