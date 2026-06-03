// Copyright 2026 Sahil Narola
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef SURFACE_WIPE_WIPE_TRAJECTORY_HPP
#define SURFACE_WIPE_WIPE_TRAJECTORY_HPP


namespace surface_wipe
{

struct Velocity2D
{
  double vx = 0.0;
  double vy = 0.0;
};

// ============================================================
// WipeTrajectory — time-based zigzag (boustrophedon) generator.
//
// Covers a rectangular area with back-and-forth strokes:
//   stroke 0:  +X  (at row 0)
//   step:      +Y  (advance one row_spacing)
//   stroke 1:  -X  (at row 1)
//   step:      +Y
//   stroke 2:  +X  (at row 2)  ... for num_passes strokes
//
// X direction alternates each stroke; Y advances between strokes.
// The last stroke has no trailing Y step.
//
// update(t) depends ONLY on absolute time t (stateless) -> unit-testable.
// Z is NOT handled here — that's the admittance node's force controller.
// ============================================================
class WipeTrajectory {
public:
  // x_len:       length of each X stroke [m]
  // row_spacing: Y distance between strokes [m]
  // num_passes:  number of X strokes (>= 1)
  // speed:       constant wipe speed [m/s], > 0
  WipeTrajectory(double x_len, double row_spacing, int num_passes, double speed)
  : speed_(speed),
    num_passes_(num_passes),
    tx_(x_len / speed),
    ts_(row_spacing / speed)
  {
    period_ = tx_ + ts_;                                   // one stroke + one step
    total_ = num_passes_ * tx_ + (num_passes_ - 1) * ts_;  // last stroke has no step
  }

  Velocity2D update(double t) const
  {
    Velocity2D v;
    if (t < 0.0 || t >= total_) {return v;}      // before start / finished -> {0,0}

    int pass = static_cast<int>(t / period_);
    if (pass >= num_passes_) {return v;}         // safety guard

    double local = t - pass * period_;
    if (local < tx_) {
      // X stroke — alternate direction each pass
      v.vx = (pass % 2 == 0) ? speed_ : -speed_;
    } else {
      // Y step between strokes (never reached on the last pass)
      v.vy = speed_;
    }
    return v;
  }

  bool   isComplete(double t) const {return t >= total_;}
  double duration() const {return total_;}

private:
  double speed_;
  int    num_passes_;
  double tx_, ts_;
  double period_, total_;
};

}  // namespace surface_wipe

#endif
