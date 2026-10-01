#pragma once
#include <chrono>

// The optional Eclipse S-CORE health hook. When the S-CORE launch manager
// starts VisionPilot it sets IDENTIFIER, and this class then loads
// libscore_vp.so with dlopen and brackets every frame with the health
// monitor's frame deadline. Without IDENTIFIER it only times the frame, so
// VisionPilot builds and runs without S-CORE.
class ScoreHook
{
public:
    // Throws std::runtime_error when IDENTIFIER is set and SCORE_VP_LIB,
    // a symbol or SCORE_VP_FRAME_MAX_MS is missing, or init fails: under the
    // launch manager that is a misconfiguration, and exiting is what makes the
    // launch manager fall back.
    ScoreHook();
    ~ScoreHook();
    ScoreHook(const ScoreHook&) = delete;
    ScoreHook& operator=(const ScoreHook&) = delete;

    bool enabled() const { return handle_ != nullptr; }
    void frame_begin();
    // Frame time in ms. Reports running once, after the first frame that
    // produced output (the offload gate has passed by then).
    double frame_end(bool produced_output);

private:
    void* handle_ = nullptr;
    int (*running_)() = nullptr;
    int (*begin_)() = nullptr;
    int (*end_)() = nullptr;
    bool reported_ = false;
    std::chrono::steady_clock::time_point t0_;
};
