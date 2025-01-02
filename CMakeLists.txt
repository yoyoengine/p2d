cmake_minimum_required(VERSION 3.10)

project(p2d)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)

add_library(p2d
    src/core.c
    src/world.c
    src/detection.c
    src/helpers.c
    src/types.c
    src/queue.c
    src/resolution.c
)

target_include_directories(p2d PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

###############
#   Lilith    #
###############

# TODO: pin tag when stable
FetchContent_Declare(
    lilith
    GIT_REPOSITORY https://github.com/yoyoengine/Lilith.git
    GIT_TAG main
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(Lilith)
target_include_directories(p2d PUBLIC ${lilith_SOURCE_DIR}/include) # Lilith.h

target_link_libraries(p2d PUBLIC lilith)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    add_compile_options(/W4 /WX)
endif()

# build with debug symbols
set(CMAKE_BUILD_TYPE Debug)