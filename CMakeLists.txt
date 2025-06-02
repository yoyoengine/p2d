cmake_minimum_required(VERSION 3.10)

project(p2d)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)

set(CMAKE_CXX_STANDARD 20)

# build with debug symbols
set(CMAKE_BUILD_TYPE Debug)

add_library(p2d
    src/log.c
    src/core.c
    src/world.c
    src/detection.c
    src/helpers.c
    src/types.c
    src/resolution.c
    src/contacts.c
    src/pairs.c
    src/collide.c
    src/joint.c
)

target_include_directories(p2d PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(p2d PRIVATE
        $<$<CONFIG:Debug>:-Wall -Wextra -Werror>
        $<$<NOT:$<CONFIG:Debug>>:-Wall -Wextra>
    )
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(p2d PRIVATE
        $<$<CONFIG:Debug>:/W4 /WX>
        $<$<NOT:$<CONFIG:Debug>>:/W4>
    )
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")

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
    target_compile_options(p2d PRIVATE -Wall -Wextra -Wpedantic -Werror)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    # target_compile_options(p2d PRIVATE /W4 /WX /wd4201)
    target_compile_options(p2d PRIVATE /wd4201)
    # msvc is too picky, so i dont care about msvc warnings
endif()

###############
#    TESTS    #
###############

option(BUILD_P2D_TESTS "Build p2d tests" OFF)

if(BUILD_P2D_TESTS)
    add_executable(p2d-test
        test/src/main.cpp
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

    # p2d
    target_link_libraries(p2d-test PRIVATE p2d)
    target_include_directories(p2d-test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

    # MSVC specific warning suppression for test target
    if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        target_compile_options(p2d-test PRIVATE /wd4576)
    endif()
endif()