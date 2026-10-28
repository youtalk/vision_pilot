#ifndef VISUALIZATION__KMS_GATE_HPP_
#define VISUALIZATION__KMS_GATE_HPP_

namespace visualization
{

enum class KmsAction { Skip, Start, Push };

// The KMS sink's one decision. The first non-empty frame starts the pipeline,
// once. After a failed start, a pipeline error or a change of frame size, the
// sink is inert until the process ends: it never restarts or tears down the
// pipeline in a running system. Pure logic, no GStreamer.
class KmsGate
{
public:
  KmsAction on_frame(bool empty, int w, int h);
  void start_result(bool ok);  // after on_frame() returned Start
  void pipeline_error();
  // True before the attempt and while the pipeline runs, false once inert.
  bool wants_frames() const { return state_ != State::Inert; }

private:
  enum class State { Waiting, Starting, Running, Inert };
  State state_ = State::Waiting;
  int w_ = 0;
  int h_ = 0;
};

}  // namespace visualization

#endif  // VISUALIZATION__KMS_GATE_HPP_
