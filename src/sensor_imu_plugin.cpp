/****************************************************************************
 *
 *   Copyright (c) 2020 Auterion AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief JSBSim IMU Plugin
 *
 * This is a IMU plugin for the JSBSim
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Roman Bapst <roman@auterion.com>
 */

#include "sensor_imu_plugin.h"


SensorImuPlugin::SensorImuPlugin(JSBSim::FGFDMExec* jsbsim)
    : SensorPlugin(jsbsim) {
  _standard_normal_distribution = std::normal_distribution<double>(0.0, 1.0);

  double sigma_bon_g = gyroscope_turn_on_bias_sigma;
  double sigma_bon_a = accelerometer_turn_on_bias_sigma;
  for (int i = 0; i < 3; ++i) {
    _gyroscope_turn_on_bias[i] = sigma_bon_g * _standard_normal_distribution(_random_generator);
    _accelerometer_turn_on_bias[i] = sigma_bon_a * _standard_normal_distribution(_random_generator);
  }
}

SensorImuPlugin::~SensorImuPlugin() {}

SensorData::Imu SensorImuPlugin::getData() {
  double sim_time = _sim_ptr->GetSimTime();
  double dt = sim_time - _last_sim_time;

  Eigen::Vector3d accel = getAccelFromJSBSim();
  Eigen::Vector3d gyro = getGyroFromJSBSim();

  addNoise(&accel, &gyro, dt);

  SensorData::Imu data;
  data.accel_b = accel;
  data.gyro_b = gyro;

  _last_sim_time = sim_time;
  return data;
}

Eigen::Vector3d SensorImuPlugin::getAccelFromJSBSim() {
  double x = _sim_ptr->GetPropertyValue("accelerations/a-pilot-x-ft_sec2");
  double y = _sim_ptr->GetPropertyValue("accelerations/a-pilot-y-ft_sec2");
  double z = _sim_ptr->GetPropertyValue("accelerations/a-pilot-z-ft_sec2");

  return Eigen::Vector3d(ftToM(x), ftToM(y), ftToM(z));
}

Eigen::Vector3d SensorImuPlugin::getGyroFromJSBSim() {
  double x = _sim_ptr->GetPropertyValue("velocities/p-rad_sec");
  double y = _sim_ptr->GetPropertyValue("velocities/q-rad_sec");
  double z = _sim_ptr->GetPropertyValue("velocities/r-rad_sec");

  return Eigen::Vector3d(x, y, z);
}

void SensorImuPlugin::addNoise(Eigen::Vector3d* linear_acceleration, Eigen::Vector3d* angular_velocity,
                               const double dt) {
  if (dt <= 0.0) return;

  // Gyrosocpe
  double tau_g = gyroscope_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_g_d = 1 / sqrt(dt) * gyroscope_noise_density;
  double sigma_b_g = gyroscope_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_g_d = sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 * (exp(-2.0 * dt / tau_g) - 1.0));
  // Compute state-transition.
  double phi_g_d = exp(-1.0 / tau_g * dt);
  // Simulate gyroscope noise processes and add them to the true angular rate.
  for (int i = 0; i < 3; ++i) {
    _gyroscope_bias[i] = phi_g_d * _gyroscope_bias[i] + sigma_b_g_d * _standard_normal_distribution(_random_generator);
    (*angular_velocity)[i] = (*angular_velocity)[i] + _gyroscope_bias[i] +
                             sigma_g_d * _standard_normal_distribution(_random_generator) + _gyroscope_turn_on_bias[i];
  }

  //   // Accelerometer
  double tau_a = accelerometer_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_a_d = 1 / sqrt(dt) * accelerometer_noise_density;
  double sigma_b_a = accelerometer_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_a_d = sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 * (exp(-2.0 * dt / tau_a) - 1.0));
  // Compute state-transition.
  double phi_a_d = exp(-1.0 / tau_a * dt);
  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  for (int i = 0; i < 3; ++i) {
    _accelerometer_bias[i] =
        phi_a_d * _accelerometer_bias[i] + sigma_b_a_d * _standard_normal_distribution(_random_generator);
    (*linear_acceleration)[i] = (*linear_acceleration)[i] + _accelerometer_bias[i] +
                                sigma_a_d * _standard_normal_distribution(_random_generator) +
                                _accelerometer_turn_on_bias[i];
  }
}