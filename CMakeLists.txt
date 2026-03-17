cmake_minimum_required(VERSION 3.16)
project(isdn3000e_lab6)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

FetchContent_Declare(
        polyscope
        GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
        GIT_TAG master
)

FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
)

FetchContent_Declare(
        libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG v2.5.0
)

FetchContent_MakeAvailable(polyscope)
FetchContent_MakeAvailable(eigen)
FetchContent_MakeAvailable(libigl)

list(PREPEND CMAKE_PREFIX_PATH "/opt/openrobots")
find_package(pinocchio REQUIRED)

add_executable(isdn3000e_lab6 main.cpp
        tasks/task1.cpp
        tasks/task1.h
        tasks/task2.cpp
        tasks/task2.h
        tasks/task3.cpp
        tasks/task3.h
)


target_link_libraries(isdn3000e_lab6
        PRIVATE
        polyscope
        Eigen3::Eigen
        igl::core
        pinocchio::pinocchio
)


set(RUNTIME_DIR $<TARGET_FILE_DIR:isdn3000e_lab6>)

add_custom_command(TARGET isdn3000e_lab6 POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/robot
        ${RUNTIME_DIR}/robot
#        COMMAND ${CMAKE_COMMAND} -E copy_directory
#        ${CMAKE_SOURCE_DIR}/assets
#        ${RUNTIME_DIR}/assets
        COMMENT "Copying robot/ and assets/ to runtime directory"
)