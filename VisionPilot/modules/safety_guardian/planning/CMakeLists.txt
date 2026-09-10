cmake_minimum_required(VERSION 3.22.1)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# lateral_planning.cpp instantiates CppAD::put_check_for_nan, which lives in
# cppad_lib rather than the header-only part. Without this every consumer of
# libplanning.a fails to link on CppAD::local::temp_file.
find_library(CPPAD_LIB NAMES cppad_lib)

include(FetchContent)

FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 5.0.0
        GIT_SHALLOW TRUE
)

# Skip building tests/docs
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)

# Install vendored Eigen headers to a private path so packaging doesn't
# collide with the system libeigen3-dev package
set(INCLUDE_INSTALL_DIR "include/visionpilot/eigen3" CACHE STRING "" FORCE)

FetchContent_MakeAvailable(eigen)

add_library(planning
        src/planning.cpp
        src/longitudinal_planning.cpp
        src/lateral_planning.cpp

)

target_include_directories(planning
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE
        ${IPOPT_INCLUDE_DIRS}
)

target_compile_definitions(planning
        PRIVATE
        HAVE_CSTDDEF
)

target_link_libraries(planning
        PUBLIC
        common
        Eigen3::Eigen
        PRIVATE
        ${IPOPT_LIBRARIES}
        $<$<BOOL:${CPPAD_LIB}>:${CPPAD_LIB}>
)