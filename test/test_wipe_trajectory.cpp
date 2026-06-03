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

#include <gtest/gtest.h>
#include "surface_wipe/wipe_trajectory.hpp"

using surface_wipe::WipeTrajectory;

TEST(WipeTrajectory, zigzag_phases) {
  // x_len=0.1, row_spacing=0.05, num_passes=3, speed=0.05
  // tx = 2.0s, ts = 1.0s, period = 3.0s, total = 3*2 + 2*1 = 8.0s
  // [0,2) +X | [2,3) +Y | [3,5) -X | [5,6) +Y | [6,8) +X
  WipeTrajectory traj(0.1, 0.05, 3, 0.05);
  EXPECT_NEAR(traj.duration(), 8.0, 1e-9);

  EXPECT_NEAR(traj.update(1.0).vx, 0.05, 1e-9);   // stroke 0: +X
  EXPECT_NEAR(traj.update(1.0).vy, 0.0, 1e-9);
  EXPECT_NEAR(traj.update(2.5).vy, 0.05, 1e-9);   // step: +Y
  EXPECT_NEAR(traj.update(2.5).vx, 0.0, 1e-9);
  EXPECT_NEAR(traj.update(4.0).vx, -0.05, 1e-9);  // stroke 1: -X
  EXPECT_NEAR(traj.update(5.5).vy, 0.05, 1e-9);   // step: +Y
  EXPECT_NEAR(traj.update(7.0).vx, 0.05, 1e-9);   // stroke 2: +X

  EXPECT_NEAR(traj.update(-1.0).vx, 0.0, 1e-9);   // before start
  EXPECT_FALSE(traj.isComplete(7.9));
  EXPECT_TRUE(traj.isComplete(8.0));
  EXPECT_NEAR(traj.update(8.0).vx, 0.0, 1e-9);    // finished -> zero
  EXPECT_NEAR(traj.update(8.0).vy, 0.0, 1e-9);
}
