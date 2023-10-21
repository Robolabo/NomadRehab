// Copyright 2022 Pixel Robotics.
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

/*
 * Author: Tony Najjar
 */

#include "synchro_drive_controller/odometry.hpp"

namespace synchro_drive_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheelbase_(0.0),
  wheel_radius_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

bool Odometry::update(double Ws, double alpha, const rclcpp::Duration & dt)
{
  static double alpha_prev = 0.0;
  /* calculate linear velocity*/
  double vl = Ws * wheel_radius_;
  double w = (alpha - alpha_prev) / dt.seconds();

  /* Integrate values */
  heading_ = alpha;
  x_ += vl * cos(heading_) * dt.seconds();
  y_ += vl * sin(heading_)* dt.seconds();

  /* Estimate speeds using a rolling mean to filter them out */
  linear_accumulator_.accumulate(vl);
  angular_accumulator_.accumulate(w);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  resetAccumulators();
}

void Odometry::setWheelParams(double wheel_radius)
{
  wheel_radius_ = wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  resetAccumulators();
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace synchro_drive_controller
