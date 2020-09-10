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
 * @brief JSBSim GPS Plugin
 *
 * This is a GPS plugin for the JSBSim
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Roman Bapst <roman@auterion.com>
 */

#include "sensor_gps_plugin.h"

SensorGpsPlugin::SensorGpsPlugin(JSBSim::FGFDMExec *jsbsim) : SensorPlugin(jsbsim) {}

SensorGpsPlugin::~SensorGpsPlugin() {}

SensorData::Gps SensorGpsPlugin::getData() {
  double sim_time = sim_ptr_->GetSimTime();
  double dt = sim_time - last_sim_time_;

  SensorData::Gps data;

  data = getGpsFromJSBSim();

  last_sim_time_ = sim_time;
  return data;
}

SensorData::Gps SensorGpsPlugin::getGpsFromJSBSim() {
  SensorData::Gps ret;
  ret.time_utc_usec = sim_ptr_->GetSimTime() * 1e6;
  ret.fix_type = 3;
  ret.latitude_deg = sim_ptr_->GetPropertyValue("position/lat-gc-deg") * 1e7;
  ret.longitude_deg = sim_ptr_->GetPropertyValue("position/long-gc-deg") * 1e7;
  ret.altitude = sim_ptr_->GetPropertyValue("position/h-sl-meters") * 1e7;
  ret.eph = 1 * 100;
  ret.epv = 2 * 100;
  ret.velocity_north = ftToM(sim_ptr_->GetPropertyValue("velocities/v-north-fps")) * 100;
  ret.velocity_east = ftToM(sim_ptr_->GetPropertyValue("velocities/v-east-fps")) * 100;
  ret.velocity_down = ftToM(sim_ptr_->GetPropertyValue("velocities/v-down-fps")) * 100;
  ret.velocity = ftToM(sim_ptr_->GetPropertyValue("velocities/ned-velocity-mag-fps")) * 100;
  ret.satellites_visible = 16;
  ret.id = 1;

  return ret;
}
