cmake_minimum_required(VERSION 3.22.1)

find_package(GTest REQUIRED)

add_executable(models_tests
        test_merged_contract.cpp
        test_detections.cpp
        test_renesas_artifacts.cpp
        test_merged_backend.cpp
)

target_link_libraries(models_tests
        PRIVATE
            models
            engine
            GTest::gtest
            GTest::gtest_main
)

target_compile_features(models_tests PRIVATE cxx_std_17)
