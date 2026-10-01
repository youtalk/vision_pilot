#include "score_hook.hpp"

#include <dlfcn.h>

#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace
{
void* symbol(void* handle, const char* name)
{
    void* s = dlsym(handle, name);
    if (s == nullptr)
    {
        throw std::runtime_error(std::string("libscore_vp.so has no ") + name);
    }
    return s;
}
}  // namespace

ScoreHook::ScoreHook()
{
    if (std::getenv("IDENTIFIER") == nullptr)
    {
        return;
    }
    const char* lib = std::getenv("SCORE_VP_LIB");
    const std::string path = lib != nullptr ? lib : "/opt/score/lib/libscore_vp.so";
    const char* max = std::getenv("SCORE_VP_FRAME_MAX_MS");
    if (max == nullptr)
    {
        throw std::runtime_error("SCORE_VP_FRAME_MAX_MS is not set");
    }
    handle_ = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle_ == nullptr)
    {
        throw std::runtime_error(std::string("dlopen: ") + dlerror());
    }
    auto init = reinterpret_cast<int (*)(uint32_t)>(symbol(handle_, "score_vp_init"));
    running_ = reinterpret_cast<int (*)()>(symbol(handle_, "score_vp_report_running"));
    begin_ = reinterpret_cast<int (*)()>(symbol(handle_, "score_vp_frame_begin"));
    end_ = reinterpret_cast<int (*)()>(symbol(handle_, "score_vp_frame_end"));
    if (init(static_cast<uint32_t>(std::stoul(max))) != 0)
    {
        throw std::runtime_error("score_vp_init failed");
    }
}

ScoreHook::~ScoreHook()
{
    // Never dlclose: the health monitor thread lives inside the library.
}

void ScoreHook::frame_begin()
{
    t0_ = std::chrono::steady_clock::now();
    if (begin_ != nullptr)
    {
        begin_();
    }
}

double ScoreHook::frame_end(bool produced_output)
{
    if (end_ != nullptr)
    {
        end_();
        if (produced_output && !reported_)
        {
            running_();
            reported_ = true;
        }
    }
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0_).count();
}
