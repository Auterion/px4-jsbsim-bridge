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
 * @brief JSBSim Magnetometer Plugin
 *
 * This is a magnetimeter plugin for the JSBSim
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Roman Bapst <roman@auterion.com>
 */

#include "sensor_mag_plugin.h"

SensorMagPlugin::SensorMagPlugin(JSBSim::FGFDMExec* jsbsim) : SensorPlugin(jsbsim) {
  _standard_normal_distribution = std::normal_distribution<double>(0.0, 1.0);
}

SensorMagPlugin::~SensorMagPlugin() {}

void SensorMagPlugin::setSensorConfigs(const TiXmlElement& configs) {
  GetConfigElement<double>(configs, "noise_density", _noise_density);
  GetConfigElement<double>(configs, "random_walk", _random_walk);
  GetConfigElement<double>(configs, "bias_correlation_time", _bias_correlation_time);
}

SensorData::Magnetometer SensorMagPlugin::getData() {
  double sim_time = _sim_ptr->GetSimTime();
  double dt = sim_time - _last_sim_time;

  Eigen::Vector3d mag_b = getMagFromJSBSim();

  addNoise(&mag_b, dt);

  SensorData::Magnetometer data;

  data.mag_b = mag_b;

  _last_sim_time = sim_time;
  return data;
}

Eigen::Vector3d SensorMagPlugin::getMagFromJSBSim() {
  double lat_deg, lon_deg, roll_rad, pitch_rad, heading_rad;

  lat_deg = _sim_ptr->GetPropertyValue("position/lat-geod-deg");
  lon_deg = _sim_ptr->GetPropertyValue("position/lon-gc-deg");
  roll_rad = _sim_ptr->GetPropertyValue("attitude/roll-rad");
  pitch_rad = _sim_ptr->GetPropertyValue("attitude/pitch-rad");
  heading_rad = wrap_pi(_sim_ptr->GetPropertyValue("attitude/heading-true-rad"));

  // Magnetic strength (10^5xnanoTesla)
  float strength_ga = 0.01f * get_mag_strength(lat_deg, lon_deg);

  // Magnetic declination and inclination (radians)
  float declination_rad = get_mag_declination(lat_deg, lon_deg) * 3.14159265f / 180;
  float inclination_rad = get_mag_inclination(lat_deg, lon_deg) * 3.14159265f / 180;

  // Magnetic filed components are calculated by http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
  float H = strength_ga * cosf(inclination_rad);
  float Z = H * tanf(inclination_rad);
  float X = H * cosf(declination_rad);
  float Y = H * sinf(declination_rad);

  Eigen::Vector3d mag_g(X, Y, Z);
  Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(heading_rad, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitY());

  Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

  Eigen::Matrix3d rotationMatrix = q.inverse().matrix();

  Eigen::Vector3d mag1 = rotationMatrix * mag_g;

  return mag1;
}

void SensorMagPlugin::addNoise(Eigen::Vector3d* magnetic_field, const double dt) {
  if (dt <= 0.0) return;

  double tau = _bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_d = 1 / sqrt(dt) * _noise_density;
  double sigma_b = _random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_d = sqrt(-sigma_b * sigma_b * tau / 2.0 * (exp(-2.0 * dt / tau) - 1.0));
  // Compute state-transition.
  double phi_d = exp(-1.0 / tau * dt);
  // Simulate magnetometer noise processes and add them to the true signal.
  for (int i = 0; i < 3; ++i) {
    _bias[i] = phi_d * _bias[i] + sigma_b_d * _standard_normal_distribution(_random_generator);
    (*magnetic_field)[i] = (*magnetic_field)[i] + _bias[i] + sigma_d * _standard_normal_distribution(_random_generator);
  }
}
