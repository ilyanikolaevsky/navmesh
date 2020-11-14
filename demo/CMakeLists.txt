add_definitions( -D_UNICODE
                 -DUNICODE)

add_executable(
    demo WIN32
    demo.cpp 
    framework.h
    resource.h
    targetver.h
    demo.ico
    demo.rc
    demo_small.ico
    )

target_link_libraries(
    demo
    navmesh
    )

target_include_directories(demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../source/
)