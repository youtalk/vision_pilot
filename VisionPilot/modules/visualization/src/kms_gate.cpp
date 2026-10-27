#include <visualization/kms_gate.hpp>

namespace visualization
{

KmsAction KmsGate::on_frame(bool empty, int w, int h)
{
  if (empty) return KmsAction::Skip;
  switch (state_) {
    case State::Waiting:
      state_ = State::Starting;
      w_ = w;
      h_ = h;
      return KmsAction::Start;
    case State::Running:
      if (w == w_ && h == h_) return KmsAction::Push;
      state_ = State::Inert;
      return KmsAction::Skip;
    default:
      return KmsAction::Skip;
  }
}

void KmsGate::start_result(bool ok)
{
  if (state_ == State::Starting) state_ = ok ? State::Running : State::Inert;
}

void KmsGate::pipeline_error()
{
  state_ = State::Inert;
}

}  // namespace visualization
