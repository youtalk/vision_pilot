#include "score_hook.hpp"

#include <dlfcn.h>

#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#define CHECK(c) do { if (!(c)) { std::printf("TEST_FAIL test_score_hook %s:%d %s\n", __FILE__, __LINE__, #c); return 1; } } while (0)

int main()
{
    // 1. No IDENTIFIER: disabled, never opens the library, still times frames.
    unsetenv("IDENTIFIER");
    setenv("SCORE_VP_LIB", "/nonexistent/libscore_vp.so", 1);
    {
        ScoreHook h;
        CHECK(!h.enabled());
        h.frame_begin();
        CHECK(h.frame_end(true) >= 0.0);
    }
    // 2. IDENTIFIER and a library: init gets the budget, running is reported once.
    setenv("IDENTIFIER", "visionpilot", 1);
    setenv("SCORE_VP_LIB", FAKE_SCORE_VP_LIB, 1);
    setenv("SCORE_VP_FRAME_MAX_MS", "80", 1);
    {
        ScoreHook h;
        CHECK(h.enabled());
        h.frame_begin(); h.frame_end(false);
        h.frame_begin(); h.frame_end(true);
        h.frame_begin(); h.frame_end(true);
        void* lib = dlopen(FAKE_SCORE_VP_LIB, RTLD_NOW | RTLD_NOLOAD);
        CHECK(lib != nullptr);
        auto calls = reinterpret_cast<int (*)(int)>(dlsym(lib, "fake_calls"));
        auto max = reinterpret_cast<uint32_t (*)()>(dlsym(lib, "fake_frame_max"));
        CHECK(calls(0) == 1 && max() == 80);
        CHECK(calls(1) == 1);
        CHECK(calls(2) == 3 && calls(3) == 3);
        // 2b. A failed running report is retried on the next produced frame.
        auto fail_running = reinterpret_cast<void (*)(int)>(dlsym(lib, "fake_fail_running"));
        ScoreHook r;
        fail_running(1);
        r.frame_begin(); r.frame_end(true);
        CHECK(calls(1) == 2);
        r.frame_begin(); r.frame_end(true);
        CHECK(calls(1) == 3);
        r.frame_begin(); r.frame_end(true);
        CHECK(calls(1) == 3);
    }
    // 3. IDENTIFIER without SCORE_VP_FRAME_MAX_MS: fail loud.
    unsetenv("SCORE_VP_FRAME_MAX_MS");
    bool threw = false;
    try { ScoreHook h; } catch (const std::runtime_error&) { threw = true; }
    CHECK(threw);
    // 4. IDENTIFIER with a missing library: fail loud.
    setenv("SCORE_VP_FRAME_MAX_MS", "80", 1);
    setenv("SCORE_VP_LIB", "/nonexistent/libscore_vp.so", 1);
    threw = false;
    try { ScoreHook h; } catch (const std::runtime_error&) { threw = true; }
    CHECK(threw);
    std::printf("TEST_PASS test_score_hook\n");
    return 0;
}
