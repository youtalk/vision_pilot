add_executable(test_dashboard
        test_dashboard.cpp
)

target_link_libraries(test_dashboard
        PRIVATE
        visualization
)

# kms_layout.cpp is compiled in directly: the logic is pure, so the test needs
# neither GStreamer nor the rest of the visualization library.
add_executable(test_kms_layout
        test_kms_layout.cpp
        ${CMAKE_SOURCE_DIR}/modules/visualization/src/kms_layout.cpp
)

target_include_directories(test_kms_layout
        PRIVATE
            ${CMAKE_SOURCE_DIR}/modules/visualization/include
)

# The KMS sink's one-attempt gate, and KmsDisplay up to the connector read.
if (KMSGST_FOUND)
    find_package(GTest REQUIRED)
    add_executable(test_kms_gate test_kms_gate.cpp)
    target_link_libraries(test_kms_gate PRIVATE visualization GTest::gtest GTest::gtest_main)
endif ()
