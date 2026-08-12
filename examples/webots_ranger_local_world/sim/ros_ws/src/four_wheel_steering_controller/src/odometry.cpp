/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Irstea
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Irstea nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "four_wheel_steering_controller/odometry.hpp"
#include <iostream>

namespace four_wheel_steering_controller
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  using std::placeholders::_5;
  using std::placeholders::_6;
  using std::placeholders::_7;
  using std::placeholders::_8;
  using std::placeholders::_9;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : update(std::bind(&Odometry::update_all, this, _1, _2, _3, _4, _5, _6, _7, _8, _9))
  , last_update_timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_(0.0)
  , linear_x_(0.0)
  , linear_y_(0.0)
  , angular_(0.0)
  , steering_track_(0.0)
  , wheel_steering_y_offset_(0.0)
  , wheel_radius_(0.0)
  , wheel_base_(0.0)
  , wheel_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_accel_acc_(velocity_rolling_window_size)
  , linear_jerk_acc_(velocity_rolling_window_size)
  , front_steer_vel_acc_(velocity_rolling_window_size)
  , rear_steer_vel_acc_(velocity_rolling_window_size)
  {
  }

  void Odometry::init(const rclcpp::Time & time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    last_update_timestamp_ = time;
  }
  
  // all-四舵轮(全)
  bool Odometry::update_all(double fl_speed, double fr_speed, double rl_speed, double rr_speed, double fl_steering, double fr_steering, double rl_steering, double rr_steering, const rclcpp::Time &time)
  {
    const double half_base = wheel_base_ / 2.0;
    const double half_track = steering_track_ / 2.0 + wheel_steering_y_offset_;
    const double wheel_x[4] = {half_base, half_base, -half_base, -half_base};
    const double wheel_y[4] = {half_track, -half_track, half_track, -half_track};
    const double wheel_speed[4] = {fl_speed, fr_speed, rl_speed, rr_speed};
    const double steering[4] = {fl_steering, fr_steering, rl_steering, rr_steering};

    double sum_vx = 0.0;
    double sum_vy = 0.0;
    double angular_numerator = 0.0;
    double angular_denominator = 0.0;
    for (size_t i = 0; i < 4; ++i)
    {
      const double velocity = wheel_radius_ * wheel_speed[i];
      const double velocity_x = velocity * cos(steering[i]);
      const double velocity_y = velocity * sin(steering[i]);
      sum_vx += velocity_x;
      sum_vy += velocity_y;
      angular_numerator += -wheel_y[i] * velocity_x + wheel_x[i] * velocity_y;
      angular_denominator += wheel_x[i] * wheel_x[i] + wheel_y[i] * wheel_y[i];
    }

    // Least-squares rigid-body twist from all four wheel velocity vectors.
    linear_x_ = sum_vx / 4.0;
    linear_y_ = sum_vy / 4.0;
    angular_ = angular_numerator / angular_denominator;
    linear_ = copysign(hypot(linear_x_, linear_y_), linear_x_);

    /// Compute x, y and heading using velocity
    const double dt = (time - last_update_timestamp_).seconds(); 
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    last_update_timestamp_ = time;
    /// Integrate odometry:
    integrateXY(linear_x_*dt, linear_y_*dt, angular_*dt, false);

    linear_accel_acc_.accumulate((linear_vel_prev_ - linear_)/dt);
    linear_vel_prev_ = linear_;
    linear_jerk_acc_.accumulate((linear_accel_prev_ - linear_accel_acc_.getRollingMean())/dt);
    linear_accel_prev_ = linear_accel_acc_.getRollingMean();
    const double front_steering = (fl_steering + fr_steering) / 2.0;
    const double rear_steering = (rl_steering + rr_steering) / 2.0;
    front_steer_vel_acc_.accumulate((front_steer_vel_prev_ - front_steering)/dt);
    front_steer_vel_prev_ = front_steering;
    rear_steer_vel_acc_.accumulate((rear_steer_vel_prev_ - rear_steering)/dt);
    rear_steer_vel_prev_ = rear_steering;
    
    return true;
  }
  
  // flrr-四舵轮(前左fl,后右rr)
  bool Odometry::update_flrr(double fl_speed, double /*fr_speed*/, double /*rl_speed*/, double rr_speed, double fl_steering, double /*fr_steering*/, double /*rl_steering*/, double rr_steering, const rclcpp::Time &time)
  {
    // 前左fl    fl_speed, fl_steering
    // 后右rr    rr_speed, rr_steering
    fl_speed = fl_speed * wheel_radius_; // 轮子转速转化为线速度
    rr_speed = rr_speed * wheel_radius_;

    // 将行进轮的速度分解到两个对角线轮子的连线上parallel及其垂直方向perpendicular
    double Vfr_parallel = fl_speed * cos(fl_steering-diagonal_angle_);
    double Vfr_perpendicular = fl_speed * sin(fl_steering-diagonal_angle_);
    double Vrl_parallel = rr_speed * cos(rr_steering-diagonal_angle_);
    double Vrl_perpendicular = rr_speed * sin(rr_steering-diagonal_angle_);

    // 对角线中心点c的速度(Vc_parallel,Vc_perpendicular,Wc)
    double Vc_parallel = (Vfr_parallel + Vrl_parallel) / 2.0;
    double Vc_perpendicular = (Vfr_perpendicular + Vrl_perpendicular) / 2.0;
    double Wc = (Vfr_perpendicular - Vrl_perpendicular) / diagonal_distance_;
    
    // 速度转换到车体坐标系下 (linear_x_, linear_y_, angular_)
    linear_x_ = Vc_parallel * cos(diagonal_angle_) - Vc_perpendicular * sin(diagonal_angle_);
    linear_y_ = Vc_parallel * sin(diagonal_angle_) + Vc_perpendicular * cos(diagonal_angle_);
    angular_ = Wc;
    linear_ =  copysign(1.0, rr_speed)*sqrt(pow(linear_x_,2)+pow(linear_y_,2));

    /// Compute x, y and heading using velocity
    const double dt = (time - last_update_timestamp_).seconds(); 
    if (dt < 0.0001)
      return false; // Interval too small to integrate with
    
    last_update_timestamp_ = time;
    
    /// Integrate odometry:
    integrateXY(linear_x_*dt, linear_y_*dt, angular_*dt, false);
    
    linear_accel_acc_.accumulate((linear_vel_prev_ - linear_)/dt);
    linear_vel_prev_ = linear_;
    linear_jerk_acc_.accumulate((linear_accel_prev_ - linear_accel_acc_.getRollingMean())/dt);
    linear_accel_prev_ = linear_accel_acc_.getRollingMean();
    front_steer_vel_acc_.accumulate((front_steer_vel_prev_ - fl_steering)/dt);
    front_steer_vel_prev_ = fl_steering;
    rear_steer_vel_acc_.accumulate((rear_steer_vel_prev_ - rr_steering)/dt);
    rear_steer_vel_prev_ = rr_steering;

    return true;
  }
  
  // frrl-四舵轮(前右fr,后左rl)
  bool Odometry::update_frrl(double /*fl_speed*/, double fr_speed, double rl_speed, double /*rr_speed*/, double /*fl_steering*/, double fr_steering, double rl_steering, double /*rr_steering*/, const rclcpp::Time &time)
  {
    // 前右fr    fr_speed, fr_steering
    // 后左rl    rl_speed, rl_steering

    fr_speed = fr_speed * wheel_radius_; // 轮子转速转化为线速度
    rl_speed = rl_speed * wheel_radius_;
    
    // 将行进轮的速度分解到两个对角线轮子的连线上parallel及其垂直方向perpendicular
    double Vfr_parallel = fr_speed * cos(fr_steering-diagonal_angle_);
    double Vfr_perpendicular = fr_speed * sin(fr_steering-diagonal_angle_);
    double Vrl_parallel = rl_speed * cos(rl_steering-diagonal_angle_);
    double Vrl_perpendicular = rl_speed * sin(rl_steering-diagonal_angle_);
    
    // 对角线中心点c的速度(Vc_parallel,Vc_perpendicular,Wc)
    double Vc_parallel = (Vfr_parallel + Vrl_parallel) / 2.0;
    double Vc_perpendicular = (Vfr_perpendicular + Vrl_perpendicular) / 2.0;
    double Wc = (Vfr_perpendicular - Vrl_perpendicular) / diagonal_distance_;
    
    // 速度转换到车体坐标系下 (linear_x_, linear_y_, angular_)
    linear_x_ = Vc_parallel * cos(diagonal_angle_) - Vc_perpendicular * sin(diagonal_angle_);
    linear_y_ = Vc_parallel * sin(diagonal_angle_) + Vc_perpendicular * cos(diagonal_angle_);
    angular_ = Wc;
    linear_ =  copysign(1.0, rl_speed)*sqrt(pow(linear_x_,2)+pow(linear_y_,2));
    
    /// Compute x, y and heading using velocity
    const double dt = (time - last_update_timestamp_).seconds(); 
    if (dt < 0.0001)
      return false; // Interval too small to integrate with
    
    last_update_timestamp_ = time;
    
    /// Integrate odometry:
    integrateXY(linear_x_*dt, linear_y_*dt, angular_*dt, false);
    
    linear_accel_acc_.accumulate((linear_vel_prev_ - linear_)/dt);
    linear_vel_prev_ = linear_;
    linear_jerk_acc_.accumulate((linear_accel_prev_ - linear_accel_acc_.getRollingMean())/dt);
    linear_accel_prev_ = linear_accel_acc_.getRollingMean();
    front_steer_vel_acc_.accumulate((front_steer_vel_prev_ - fr_steering)/dt);
    front_steer_vel_prev_ = fr_steering;
    rear_steer_vel_acc_.accumulate((rear_steer_vel_prev_ - rl_steering)/dt);
    rear_steer_vel_prev_ = rl_steering;
    
    return true;
  }
  
  void Odometry::setUpdateFunction(std::string chassis_type)
  {
    if (chassis_type == "flrr") // flrr-四舵轮(前左fl,后右rr)
    {
      update = std::bind(&Odometry::update_flrr, this, _1, _2, _3, _4, _5, _6, _7, _8, _9);
    }
    else if (chassis_type == "frrl") // frrl-四舵轮(前右fr,后左rl)
    {
      update = std::bind(&Odometry::update_frrl, this, _1, _2, _3, _4, _5, _6, _7, _8, _9);
      diagonal_angle_ *= -1.0; // 符号取反
    }
    else // all-四舵轮(全)
    {
      update = std::bind(&Odometry::update_all, this, _1, _2, _3, _4, _5, _6, _7, _8, _9);
    }
  }
  
  void Odometry::setWheelParams(double steering_track, double wheel_steering_y_offset, double wheel_radius, double wheel_base)
  {
    steering_track_   = steering_track;
    wheel_steering_y_offset_ = wheel_steering_y_offset;
    wheel_radius_     = wheel_radius;
    wheel_base_       = wheel_base;
    
    diagonal_angle_ = atan2(steering_track_, wheel_base_);
    diagonal_distance_ = hypot(steering_track_, wheel_base_);
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateXY(double linear_x, double linear_y, double angular, bool pivot_turn)
  {
    double delta_x = 0;
    double delta_y = 0;

    if(pivot_turn == false)
    {
      delta_x = linear_x*cos(heading_) - linear_y*sin(heading_);
      delta_y = linear_x*sin(heading_) + linear_y*cos(heading_);
    }
    else
    {
      delta_x = 0;
      delta_y = 0;
    }

    x_ += delta_x;
    y_ += delta_y;
    heading_ += angular;
  }

  void Odometry::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
  }

  void Odometry::integrateExact(double linear, double angular)
  {
    if (fabs(angular) < 1e-6)
      integrateRungeKutta2(linear, angular);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linear/angular;
      heading_ += angular;
      x_       +=  r * (sin(heading_) - sin(heading_old));
      y_       += -r * (cos(heading_) - cos(heading_old));
    }
  }

  void Odometry::resetAccumulators()
  {
    linear_accel_acc_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    linear_jerk_acc_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    front_steer_vel_acc_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    rear_steer_vel_acc_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  }

} // namespace four_wheel_steering_controller
