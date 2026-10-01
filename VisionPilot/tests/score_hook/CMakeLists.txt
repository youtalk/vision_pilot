add_library(fake_score_vp SHARED fake_score_vp.c)
set_target_properties(fake_score_vp PROPERTIES LINKER_LANGUAGE C)
enable_language(C)

add_executable(test_score_hook
        test_score_hook.cpp
        ${CMAKE_SOURCE_DIR}/app/score_hook.cpp
)
target_include_directories(test_score_hook PRIVATE ${CMAKE_SOURCE_DIR}/app)
target_link_libraries(test_score_hook PRIVATE ${CMAKE_DL_LIBS})
target_compile_definitions(test_score_hook PRIVATE FAKE_SCORE_VP_LIB="$<TARGET_FILE:fake_score_vp>")
add_dependencies(test_score_hook fake_score_vp)
