#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/homing.hpp>
#include <franka_msgs/action/move.hpp>
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace panda_gripper_sim {
using Move = franka_msgs::action::Move;
using Grasp = franka_msgs::action::Grasp;
using Homing = franka_msgs::action::Homing;
using GripperCommand = control_msgs::action::GripperCommand;
template<class Action> using Goal = rclcpp_action::ServerGoalHandle<Action>;
using Handle = std::variant<std::monostate, std::shared_ptr<Goal<Move>>,
    std::shared_ptr<Goal<Grasp>>, std::shared_ptr<Goal<Homing>>,
    std::shared_ptr<Goal<GripperCommand>>>;
enum class Mode { Move, Grasp, Homing, Standard };
struct Request {
  Mode mode;
  double width, speed, force, inner = 0.005, outer = 0.005;
};
struct Sample {
  double time = 0, width = 0.08, velocity = 0, effort = 0;
  std::array<double, 2> position{0.04, 0.04}, speed{0, 0};
};

class Hand : public std::enable_shared_from_this<Hand> {
 public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model) {
    node_ = node;
    model_ = model;
    const std::string parameter = "mujoco_plugins." + node->get_sub_namespace() + ".";
    node->get_parameter_or(parameter + "position_gain", kp_, 3000.0);
    node->get_parameter_or(parameter + "velocity_gain", kd_, 12.0);
    node->get_parameter_or(parameter + "default_speed", default_speed_, 0.05);
    node->get_parameter_or(parameter + "move_force", move_force_, 30.0);
    node->get_parameter_or(parameter + "acceleration", acceleration_, 0.5);
    if (!(kp_ > 0 && kd_ > 0 && default_speed_ > 0 && default_speed_ <= 0.1 &&
          move_force_ >= 30 && move_force_ <= 70 && acceleration_ > 0)) return false;
    for (int i = 0; i < 2; ++i) {
      const auto name = "panda_finger_joint" + std::to_string(i + 1);
      joint_[i] = mj_name2id(model, mjOBJ_JOINT, name.c_str());
      motor_[i] = mj_name2id(model, mjOBJ_ACTUATOR, name.c_str());
      if (joint_[i] < 0 || motor_[i] < 0) return false;
      q_[i] = model->jnt_qposadr[joint_[i]];
      v_[i] = model->jnt_dofadr[joint_[i]];
      body_[i] = model->jnt_bodyid[joint_[i]];
      contact_force_[i].assign(model->nbody, 0);
    }
    robot_root_ = mj_name2id(model, mjOBJ_BODY, "panda_link0");
    prefix_ = node->get_effective_namespace();
    const std::string parent = node->get_namespace();
    states_ = node->create_publisher<sensor_msgs::msg::JointState>(parent + "/joint_states", 10);
    auto self = shared_from_this();
    move_ = server<Move>("move", [self](const auto& g) -> std::optional<Request> {
      Request r{Mode::Move, g.width, g.speed, self->move_force_};
      return self->valid(r) ? std::optional<Request>(r) : std::nullopt;
    });
    grasp_ = server<Grasp>("grasp", [self](const auto& g) -> std::optional<Request> {
      Request r{Mode::Grasp, g.width, g.speed, g.force, g.epsilon.inner, g.epsilon.outer};
      return self->valid(r) ? std::optional<Request>(r) : std::nullopt;
    });
    homing_ = server<Homing>("homing", [self](const auto&) -> std::optional<Request> {
      return Request{Mode::Homing, 0.0, self->default_speed_, self->move_force_};
    });
    standard_ = server<GripperCommand>("gripper_action", [self](const auto& g) -> std::optional<Request> {
      Request r{Mode::Standard, 2 * g.command.position, self->default_speed_, g.command.max_effort};
      if (!std::isfinite(r.force) || r.force < 0 || r.force > 70) return std::nullopt;
      if (r.force == 0) { r.force = self->move_force_; r.mode = Mode::Move; }
      return self->valid(r) ? std::optional<Request>(r) : std::nullopt;
    });
    stop_ = node->create_service<std_srvs::srv::Trigger>(prefix_ + "/stop",
      [self](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(self->mutex_);
        self->stop_requested_ = true;
        response->success = !self->shutdown_;
        response->message = "Stop requested";
      });
    timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [self]() { self->publish(); });
    return true;
  }

  void step(mjData* data) {
    std::lock_guard<std::mutex> lock(mutex_);
    sample_.time = data->time;
    sample_.effort = 0;
    for (int i = 0; i < 2; ++i) {
      sample_.position[i] = data->qpos[q_[i]];
      sample_.speed[i] = data->qvel[v_[i]];
      sample_.effort += std::abs(data->actuator_force[motor_[i]]) * 0.5;
    }
    sample_.width = sample_.position[0] + sample_.position[1];
    sample_.velocity = sample_.speed[0] + sample_.speed[1];
    const double dt = model_->opt.timestep;
    if (start_requested_) {
      start_requested_ = false;
      active_ = true;
      holding_grasp_ = false;
      desired_ = sample_.width;
      desired_velocity_ = 0;
      stable_ = elapsed_ = 0;
      homing_open_ = false;
      if (request_.mode == Mode::Standard) {
        request_.mode = request_.width < sample_.width - 0.0001 ? Mode::Grasp : Mode::Move;
      }
    }
    if (stop_requested_ || shutdown_) {
      stop_requested_ = false;
      holding_grasp_ = false;
      desired_ = sample_.width;
      desired_velocity_ = 0;
      if (active_) finish(false, "Stopped");
    }
    if (active_) {
      elapsed_ += dt;
      const bool grasping = request_.mode == Mode::Grasp;
      const double target = grasping ? 0.0 :
        (request_.mode == Mode::Homing ? (homing_open_ ? 0.08 : 0.0) : request_.width);
      const double delta = target - desired_;
      const double velocity = std::copysign(std::min(request_.speed,
          std::sqrt(2 * acceleration_ * std::abs(delta))), delta);
      desired_velocity_ += std::clamp(velocity - desired_velocity_, -acceleration_ * dt, acceleration_ * dt);
      const double step = desired_velocity_ * dt;
      if (std::abs(step) >= std::abs(delta) && step * delta >= 0) {
        desired_ = target;
        desired_velocity_ = 0;
      } else {
        desired_ = std::clamp(desired_ + step, 0.0, 0.08);
      }
      const bool reached = std::abs(sample_.width - target) < 0.0002;
      const bool obstructed = std::abs(sample_.width - desired_) > 0.0008;
      if (std::abs(sample_.velocity) < 0.001 && (reached || obstructed)) stable_ += dt;
      else stable_ = 0;
      if (stable_ >= 0.15) {
        if (request_.mode == Mode::Homing && reached && !homing_open_) {
          homing_open_ = true;
          stable_ = 0;
        } else if (grasping) {
          const bool in_band = sample_.width > request_.width - request_.inner &&
                               sample_.width < request_.width + request_.outer;
          const bool success = in_band && bilateral_contact(data);
          holding_grasp_ = success;
          finish(success, success ? "" : "No bilateral object contact within grasp tolerance");
        } else {
          finish(reached, reached ? "" : "Motion obstructed");
        }
      } else if (elapsed_ > 0.16 / request_.speed + 3.0) {
        finish(false, "Motion timed out");
      }
    }
    const double force = holding_grasp_ || active_ ? request_.force : move_force_;
    for (int i = 0; i < 2; ++i) {
      const double effort = holding_grasp_ ? -force :
          kp_ * (desired_ * 0.5 - sample_.position[i]) +
          kd_ * (desired_velocity_ * 0.5 - sample_.speed[i]);
      data->ctrl[motor_[i]] = shutdown_ ? 0 : std::clamp(effort, -force, force);
    }
  }

  void reset(mjData* data) {
    std::lock_guard<std::mutex> lock(mutex_);
    sample_.time = data->time;
    for (int i = 0; i < 2; ++i) {
      sample_.position[i] = data->qpos[q_[i]];
      sample_.speed[i] = data->qvel[v_[i]];
    }
    sample_.width = sample_.position[0] + sample_.position[1];
    desired_ = data->qpos[q_[0]] + data->qpos[q_[1]];
    desired_velocity_ = 0;
    holding_grasp_ = start_requested_ = stop_requested_ = false;
    if (busy_) finish(false, "Simulation reset");
    desired_ = sample_.width;
    for (int motor : motor_) data->ctrl[motor] = 0;
  }

  void cleanup() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_ = true;
    }
    if (timer_) timer_->cancel();
    timer_.reset();
    move_.reset(); grasp_.reset(); homing_.reset(); standard_.reset(); stop_.reset();
  }

 private:
  bool valid(const Request& r) const {
    return std::isfinite(r.width) && r.width >= 0 && r.width <= 0.08 &&
      std::isfinite(r.speed) && r.speed > 0 && r.speed <= 0.1 &&
      std::isfinite(r.force) && r.force >= 30 && r.force <= 70 &&
      std::isfinite(r.inner) && r.inner >= 0 && std::isfinite(r.outer) && r.outer >= 0;
  }

  template<class Action, class Parser>
  typename rclcpp_action::Server<Action>::SharedPtr server(const std::string& name, Parser parse) {
    auto self = shared_from_this();
    return rclcpp_action::create_server<Action>(node_, prefix_ + "/" + name,
      [self, parse](const auto&, const auto& goal) {
        std::lock_guard<std::mutex> lock(self->mutex_);
        if (self->busy_ || self->shutdown_ || !parse(*goal)) return rclcpp_action::GoalResponse::REJECT;
        self->busy_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [self](const auto&) {
        std::lock_guard<std::mutex> lock(self->mutex_);
        self->stop_requested_ = true;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [self, parse](const auto& handle) {
        std::lock_guard<std::mutex> lock(self->mutex_);
        self->handle_ = handle;
        self->request_ = *parse(*handle->get_goal());
        self->start_requested_ = true;
      });
  }

  bool bilateral_contact(mjData* data) {
    for (auto& forces : contact_force_) std::fill(forces.begin(), forces.end(), 0);
    for (int c = 0; c < data->ncon; ++c) {
      const auto& contact = data->contact[c];
      const int a = model_->geom_bodyid[contact.geom[0]], b = model_->geom_bodyid[contact.geom[1]];
      for (int i = 0; i < 2; ++i) {
        const int other = a == body_[i] ? b : (b == body_[i] ? a : 0);
        bool robot = false;
        for (int ancestor = other; ancestor > 0; ancestor = model_->body_parentid[ancestor])
          if (ancestor == robot_root_) robot = true;
        if (other > 0 && !robot) {
          mjtNum force[6];
          mj_contactForce(model_, data, c, force);
          contact_force_[i][other] += std::max<mjtNum>(0, force[0]);
        }
      }
    }
    for (int body = 1; body < model_->nbody; ++body)
      if (contact_force_[0][body] > 0.5 && contact_force_[1][body] > 0.5) return true;
    return false;
  }

  void finish(bool success, const std::string& error) {
    active_ = false;
    completed_ = true;
    success_ = success;
    error_ = error;
    desired_ = success && request_.mode == Mode::Move ? request_.width :
      (success && request_.mode == Mode::Homing ? 0.08 : sample_.width);
    desired_velocity_ = 0;
  }

  void publish() {
    Sample sample;
    Handle handle;
    bool completed, success;
    std::string error;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (shutdown_) return;
      sample = sample_;
      handle = handle_;
      completed = completed_;
      success = success_;
      error = error_;
      if (completed) {
        completed_ = busy_ = false;
        handle_ = std::monostate{};
      }
    }
    // Remove only sub-micrometre solver roundoff at the mechanical stops.
    for (double& position : sample.position) {
      if (position >= -1e-7 && position <= 0.0400001) position = std::clamp(position, 0.0, 0.04);
    }
    sample.width = sample.position[0] + sample.position[1];
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(sample.time * 1e9));
    msg.name = {"panda_finger_joint1", "panda_finger_joint2"};
    msg.position.assign(sample.position.begin(), sample.position.end());
    msg.velocity.assign(sample.speed.begin(), sample.speed.end());
    msg.effort = {sample.effort, sample.effort};
    states_->publish(msg);
    std::visit([&](const auto& goal) {
      using T = std::decay_t<decltype(goal)>;
      if constexpr (!std::is_same_v<T, std::monostate>) {
        using Action = std::conditional_t<std::is_same_v<T, std::shared_ptr<Goal<Move>>>, Move,
          std::conditional_t<std::is_same_v<T, std::shared_ptr<Goal<Grasp>>>, Grasp,
          std::conditional_t<std::is_same_v<T, std::shared_ptr<Goal<Homing>>>, Homing, GripperCommand>>>;
        auto feedback = std::make_shared<typename Action::Feedback>();
        if constexpr (std::is_same_v<Action, GripperCommand>) {
          feedback->position = sample.width * 0.5;
          feedback->effort = sample.effort;
          feedback->stalled = false;
          feedback->reached_goal = completed && success;
        } else feedback->current_width = sample.width;
        if (!completed) { goal->publish_feedback(feedback); return; }
        auto result = std::make_shared<typename Action::Result>();
        if constexpr (std::is_same_v<Action, GripperCommand>) {
          result->position = sample.width * 0.5;
          result->effort = sample.effort;
          result->stalled = !success && std::abs(sample.velocity) < 0.001;
          result->reached_goal = success;
        } else { result->success = success; result->error = error; }
        if (goal->is_canceling()) goal->canceled(result);
        else if (success) goal->succeed(result);
        else goal->abort(result);
      }
    }, handle);
  }

  rclcpp::Node::SharedPtr node_;
  const mjModel* model_ = nullptr;
  std::string prefix_;
  std::mutex mutex_;
  std::array<int, 2> joint_, motor_, q_, v_, body_;
  int robot_root_ = -1;
  Sample sample_;
  Request request_{Mode::Move, 0.08, 0.05, 30};
  Handle handle_;
  double desired_ = 0.08, desired_velocity_ = 0, stable_ = 0, elapsed_ = 0;
  double kp_ = 3000, kd_ = 12, default_speed_ = 0.05, move_force_ = 30, acceleration_ = 0.5;
  bool busy_ = false, active_ = false, completed_ = false, success_ = false;
  std::array<std::vector<mjtNum>, 2> contact_force_;
  bool start_requested_ = false, stop_requested_ = false, holding_grasp_ = false;
  bool homing_open_ = false, shutdown_ = false;
  std::string error_;
  rclcpp_action::Server<Move>::SharedPtr move_;
  rclcpp_action::Server<Grasp>::SharedPtr grasp_;
  rclcpp_action::Server<Homing>::SharedPtr homing_;
  rclcpp_action::Server<GripperCommand>::SharedPtr standard_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr states_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class FrankaHand : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase {
 public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData*) override {
    hand_ = std::make_shared<Hand>();
    return hand_->init(node, model);
  }
  void pre_step(mjData* data) override { hand_->step(data); }
  void on_reset(mjData* data) override { hand_->reset(data); }
  void cleanup() override { if (hand_) hand_->cleanup(); }
 private:
  std::shared_ptr<Hand> hand_;
};
}
PLUGINLIB_EXPORT_CLASS(panda_gripper_sim::FrankaHand, mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
