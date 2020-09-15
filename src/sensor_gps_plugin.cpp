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
  double sim_time = _sim_ptr->GetSimTime();
  double dt = sim_time - _last_sim_time;

  SensorData::Gps data;

  data = getGpsFromJSBSim();

  _last_sim_time = sim_time;
  return data;
}

SensorData::Gps SensorGpsPlugin::getGpsFromJSBSim() {
  SensorData::Gps ret;
  ret.time_utc_usec = _sim_ptr->GetSimTime() * 1e6;
  ret.fix_type = 3;
  ret.latitude_deg = _sim_ptr->GetPropertyValue("position/lat-gc-deg") * 1e7;
  ret.longitude_deg = _sim_ptr->GetPropertyValue("position/long-gc-deg") * 1e7;
  ret.altitude = _sim_ptr->GetPropertyValue("position/h-sl-meters") * 1e7;
  ret.eph = 1 * 100;
  ret.epv = 2 * 100;
  ret.velocity_north = ftToM(_sim_ptr->GetPropertyValue("velocities/v-north-fps")) * 100;
  ret.velocity_east = ftToM(_sim_ptr->GetPropertyValue("velocities/v-east-fps")) * 100;
  ret.velocity_down = ftToM(_sim_ptr->GetPropertyValue("velocities/v-down-fps")) * 100;
  ret.velocity = ftToM(_sim_ptr->GetPropertyValue("velocities/ned-velocity-mag-fps")) * 100;
  ret.satellites_visible = 16;
  ret.id = 1;

  return ret;
}
