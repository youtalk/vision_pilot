/* Fake libscore_vp.so: counts the calls so the test can read them back. */
#include <stdint.h>
static int calls[4];
static uint32_t frame_max;
static int running_failures;
int score_vp_init(uint32_t ms) { frame_max = ms; ++calls[0]; return 0; }
int score_vp_report_running(void) { ++calls[1]; if (running_failures > 0) { --running_failures; return -1; } return 0; }
int score_vp_frame_begin(void) { ++calls[2]; return 0; }
int score_vp_frame_end(void) { ++calls[3]; return 0; }
int fake_calls(int which) { return calls[which]; }
uint32_t fake_frame_max(void) { return frame_max; }
void fake_fail_running(int n) { running_failures = n; }
