cmake_minimum_required(VERSION 3.10)

project(p2d-test)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)

add_executable(p2d-test
    src/main.cpp
)

# SDL3
FetchContent_Declare(
    SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG        preview-3.1.6
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(SDL3)
target_link_libraries(p2d-test PRIVATE SDL3-shared)
target_include_directories(p2d-test PRIVATE ${SDL3_SOURCE_DIR}/include)

# add and link against p2d, one dir above is CMakeLists.txt root
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. p2d)
target_link_libraries(p2d-test PRIVATE p2d)
target_include_directories(p2d-test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)