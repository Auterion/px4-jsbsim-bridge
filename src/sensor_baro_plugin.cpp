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
 * @brief JSBSim Barometer Plugin
 *
 * This is a Barometer plugin for the JSBSim
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Roman Bapst <roman@auterion.com>
 */

#include "sensor_baro_plugin.h"

SensorBaroPlugin::SensorBaroPlugin(JSBSim::FGFDMExec* jsbsim) : SensorPlugin(jsbsim) {
  _standard_normal_distribution = std::normal_distribution<double>(0.0, 1.0);
}

SensorBaroPlugin::~SensorBaroPlugin() {}

void SensorBaroPlugin::setSensorConfigs(const TiXmlElement& configs) {
  GetConfigElement<double>(configs, "drift_pa", _baro_drift_pa);
  GetConfigElement<double>(configs, "rnd_y2", _baro_rnd_y2);
}

SensorData::Barometer SensorBaroPlugin::getData() {
  double sim_time = _sim_ptr->GetSimTime();
  double dt = sim_time - _last_sim_time;

  double temperature = getAirTemperature() + 0.2 * _standard_normal_distribution(_random_generator);
  double abs_pressure = getAirPressure() + 0.2 * _standard_normal_distribution(_random_generator);
  double pressure_alt = getPressureAltitude() + 0.2 * _standard_normal_distribution(_random_generator);

  SensorData::Barometer data;

  data.temperature = temperature;
  data.abs_pressure = abs_pressure;
  data.pressure_alt = pressure_alt;

  _last_sim_time = sim_time;
  return data;
}

float SensorBaroPlugin::getAirTemperature() { return rankineToCelsius(_sim_ptr->GetPropertyValue("atmosphere/T-R")); }

float SensorBaroPlugin::getPressureAltitude() {
  return ftToM(_sim_ptr->GetPropertyValue("atmosphere/pressure-altitude"));
}

float SensorBaroPlugin::getAirPressure() {
  // calculate millibar
  return psfToBar(_sim_ptr->GetPropertyValue("atmosphere/P-psf")) * 1000;
}
