add_executable(test_dashboard
        test_dashboard.cpp
)

target_link_libraries(test_dashboard
        PRIVATE
        visualization
)
