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

// ============================================================
// wipe_node.cpp — the "brain" of the surface-wiping demo.
//
// Runs a 4-state machine (IDLE -> APPROACH -> WIPE -> RETRACT) and, on a
// timer, publishes a WipeSetpoint telling the admittance node what to do:
//   APPROACH: descend slowly (velocity-commanded Z, no force control)
//   WIPE:     hand Z to force control (target force) + lateral zigzag
//   RETRACT:  lift off, then return to IDLE
//
// This node owns the WipeTrajectory (lateral motion) but NOT the PI force
// controller — force control executes in the admittance node, because that
// is where the arm is actually commanded.
//
// Sign convention (hardware-verified):
//   wrench_corrected Fz is NEGATIVE during surface contact (pressing up).
//   Contact detection: fz < -contact_threshold_ (e.g. fz < -2.0 N)
//   Approach velocity: negative = down toward surface.
//   Retract velocity:  positive = up away from surface.
//
// Validated results (June 2026):
//   Kp=0.001, Ki=0.01, F_desired=5N, wipe_speed=0.02 m/s
//   Steady-state force tracking: [-5.5, -4.5] N (target: -5 N)
//   Overshoot at direction reversals: up to -7 N (Ki integral windup)
//   Undershoot during lateral disturbance: down to -3 N
// ============================================================

#include <chrono>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <wipe_msgs/msg/wipe_setpoint.hpp>

#include "surface_wipe/wipe_trajectory.hpp"

using namespace std::chrono_literals;

class WipeNode : public rclcpp::Node
{
public:
  WipeNode()
  : Node("wipe_node")
  {
    // --- Parameters (tune without recompiling) ---
    x_len_            = declare_parameter("x_len", 0.10);            // m, stroke length
    row_spacing_      = declare_parameter("row_spacing", 0.05);      // m, Y step between strokes
    num_passes_       = declare_parameter("num_passes", 3);          // number of strokes
    wipe_speed_       = declare_parameter("wipe_speed", 0.02);       // m/s, lateral speed
    approach_speed_   = declare_parameter("approach_speed", 0.01);   // m/s, descent (magnitude)
    contact_threshold_= declare_parameter("contact_threshold", 2.0); // N, Fz to trip contact
    force_desired_    = declare_parameter("force_desired", 5.0);     // N, target press during wipe
    retract_speed_    = declare_parameter("retract_speed", 0.02);    // m/s, lift (magnitude)
    retract_duration_ = declare_parameter("retract_duration", 1.0);  // s, how long to lift

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "approach_speed")      approach_speed_ = p.as_double();
          else if (p.get_name() == "force_desired")  force_desired_  = p.as_double();
          else if (p.get_name() == "contact_threshold") contact_threshold_ = p.as_double();
          // add others you want live-tunable
        }
        return result;
      });
      
    // Build the lateral trajectory generator from params.
    traj_ = std::make_unique<surface_wipe::WipeTrajectory>(
      x_len_, row_spacing_, num_passes_, wipe_speed_);

    // --- Publisher: the command channel to the admittance node ---
    setpoint_pub_ = create_publisher<wipe_msgs::msg::WipeSetpoint>(
      "~/wipe_setpoint", 10);

    // --- Subscriber: read back corrected wrench to detect contact ---
    // Callback is trivial: just store the latest Fz. No logic here.
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/admittance_node/wrench_corrected",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(fz_mutex_);
        latest_fz_ = msg->wrench.force.z;
      });

    // --- Service: ~/start flips IDLE -> APPROACH ---
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        if (state_ == State::IDLE) {
          state_ = State::APPROACH;
          res->success = true;
          res->message = "Wipe started. Approaching.";
        } else {
          res->success = false;
          res->message = "Already running.";   // guard against double-start
        }
      });

    // --- Timer: drives the state machine at 50 Hz ---
    timer_ = create_wall_timer(20ms, [this]() { onTimer(); });

    RCLCPP_INFO(get_logger(), "wipe_node ready (IDLE). Call ~/start to begin.");
  }

private:
  enum class State { IDLE, APPROACH, WIPE, RETRACT };

  // The heart of the brain: evaluate state, build setpoint, publish.
  void onTimer()
  {
    // Snapshot the shared force value under the lock, then release.
    double fz;
    {
      std::lock_guard<std::mutex> lock(fz_mutex_);
      fz = latest_fz_;
    }

    wipe_msgs::msg::WipeSetpoint msg;
    msg.header.stamp = now();

    switch (state_) {
      case State::IDLE:
        // Tell admittance node to run its normal 6-DOF behavior.
        msg.mode = wipe_msgs::msg::WipeSetpoint::IDLE;
        msg.z_force_control = false;
        // velocities default to 0
        break;

      case State::APPROACH:
        // Descend slowly, Z velocity-commanded (no force control yet).
        msg.mode = wipe_msgs::msg::WipeSetpoint::APPROACH;
        msg.z_force_control = false;
        msg.velocity_z = -approach_speed_;   // negative = down
        // Contact check: only transition checked against force.
        if (fz < -contact_threshold_) {
          t_contact_ = now();                // t=0 reference for the trajectory
          state_ = State::WIPE;
          RCLCPP_INFO(get_logger(), "Contact detected (Fz=%.2f N). Wiping.", fz);
        }
        break;

      case State::WIPE: {
        // Z handed to force control; X/Y from the zigzag trajectory.
        const double t = (now() - t_contact_).seconds();
        msg.mode = wipe_msgs::msg::WipeSetpoint::WIPE;
        msg.z_force_control = true;
        msg.force_desired_z = force_desired_;
        const auto vel = traj_->update(t);
        msg.velocity_x = vel.vx;
        msg.velocity_y = vel.vy;
        if (traj_->isComplete(t)) {
          t_retract_start_ = now();
          state_ = State::RETRACT;
          RCLCPP_INFO(get_logger(), "Wipe complete. Retracting.");
        }
        break;
      }

      case State::RETRACT: {
        // Lift off (Z velocity-commanded, positive = up) for a fixed time.
        msg.mode = wipe_msgs::msg::WipeSetpoint::RETRACT;
        msg.z_force_control = false;
        msg.velocity_z = retract_speed_;     // positive = up
        const double t = (now() - t_retract_start_).seconds();
        if (t > retract_duration_) {
          state_ = State::IDLE;
          RCLCPP_INFO(get_logger(), "Retract done. Back to IDLE.");
        }
        break;
      }
    }

    setpoint_pub_->publish(msg);
  }

  // --- ROS interfaces ---
  rclcpp::Publisher<wipe_msgs::msg::WipeSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // --- Trajectory generator (lateral motion) ---
  std::unique_ptr<surface_wipe::WipeTrajectory> traj_;

  // --- State machine ---
  State state_ = State::IDLE;
  rclcpp::Time t_contact_;        // set when entering WIPE
  rclcpp::Time t_retract_start_;  // set when entering RETRACT

  // --- Shared force reading (written in sub callback, read in timer) ---
  std::mutex fz_mutex_;
  double latest_fz_ = 0.0;

  // --- Parameters ---
  double x_len_, row_spacing_, wipe_speed_;
  int    num_passes_;
  double approach_speed_, contact_threshold_, force_desired_;
  double retract_speed_, retract_duration_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WipeNode>());
  rclcpp::shutdown();
  return 0;
}