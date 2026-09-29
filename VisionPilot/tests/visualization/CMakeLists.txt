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
