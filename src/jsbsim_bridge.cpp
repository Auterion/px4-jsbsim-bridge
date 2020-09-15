
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
 * @file jsbsim_bridge.cpp
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Roman Bapst <roman@auterion.com>
 *
 * Mavlink HIL message interface to FlightGear and PX4
 */

#include "jsbsim_bridge.h"

JSBSimBridge::JSBSimBridge(JSBSim::FGFDMExec *fdmexec, std::string &path)
    : _fdmexec(fdmexec), realtime(true), result(true), dt(0.004) {
  TiXmlDocument doc(path);
  if (!doc.LoadFile()) {
    std::cerr << "Could not load actuator configs from configuration file: " << path << std::endl;
    return;
  }
  TiXmlHandle config(doc.RootElement());

  _fdmexec->Setdt(dt);
  _fdmexec->RunIC();

  // Configure Mavlink HIL interface
  _mavlink_interface = std::make_unique<MavlinkInterface>();
  SetMavlinkInterfaceConfigs(_mavlink_interface, config);

  _mavlink_interface->Load();

  // Instantiate sensors
  if (CheckConfigElement(config, "sensors", "imu")) {
    _imu_sensor = std::make_unique<SensorImuPlugin>(_fdmexec);
  } else {
    std::cerr << "Could not find IMU sensor " << std::endl;
    return;
  }

  if (CheckConfigElement(config, "sensors", "gps")) {
    _gps_sensor = std::make_unique<SensorGpsPlugin>(_fdmexec);
    _gps_sensor->setUpdateRate(1.0);
  }

  if (CheckConfigElement(config, "sensors", "barometer")) {
    _baro_sensor = std::make_unique<SensorBaroPlugin>(_fdmexec);
  }

  if (CheckConfigElement(config, "sensors", "magnetometer")) {
    _mag_sensor = std::make_unique<SensorMagPlugin>(_fdmexec);
  }

  if (CheckConfigElement(config, "sensors", "airspeed")) {
    _airspeed_sensor = std::make_unique<SensorAirspeedPlugin>(_fdmexec);
  }

  _actuators = std::make_unique<ActuatorPlugin>(_fdmexec);
  _actuators->SetActuatorConfigs(config);

  _last_step_time = std::chrono::system_clock::now();
}

JSBSimBridge::~JSBSimBridge() {}

bool JSBSimBridge::CheckConfigElement(TiXmlHandle &config, std::string group, std::string name) {
  TiXmlElement *group_element = config.FirstChild(group).Element();
  if (!group_element) {
    return false;
  }

  TiXmlElement *e = group_element->FirstChildElement(name);
  return e != nullptr;
}

bool JSBSimBridge::SetMavlinkInterfaceConfigs(std::unique_ptr<MavlinkInterface> &interface, TiXmlHandle &config) {
  TiXmlElement *mavlink_configs = config.FirstChild("mavlink_interface").Element();

  if (!mavlink_configs) return true;  // Nothing to set

  if (mavlink_configs->FirstChildElement("tcp_port")) {
    interface->SetMavlinkTcpPort(std::stoi(mavlink_configs->FirstChildElement("tcp_port")->GetText()));
  } else {
    interface->SetMavlinkTcpPort(kDefaultSITLTcpPort);
  }
  interface->SetUseTcp(true);
  interface->SetEnableLockstep(true);

  return true;
}

void JSBSimBridge::Run() {
  // Get Simulation time from JSBSim
  auto current_time = std::chrono::system_clock::now();
  double simtime = _fdmexec->GetSimTime();

  // Update sensor messages
  if (_imu_sensor && _imu_sensor->updated()) {
    // Only send sensor messages when the imu sensor is updated.
    // This is needed for lockstep
    _mavlink_interface->UpdateIMU(_imu_sensor->getData());

    if (_mag_sensor && _mag_sensor->updated()) {
      _mavlink_interface->UpdateMag(_mag_sensor->getData());
    }

    if (_baro_sensor && _baro_sensor->updated()) {
      _mavlink_interface->UpdateBarometer(_baro_sensor->getData());
    }

    if (_airspeed_sensor && _airspeed_sensor->updated()) {
      _mavlink_interface->UpdateAirspeed(_airspeed_sensor->getData());
    }

    // Send Mavlink HIL_SENSOR message
    _mavlink_interface->SendSensorMessages(simtime * 1e6);
  }

  // Send Mavlink HIL_GPS message
  if (_gps_sensor && _gps_sensor->updated()) {
    _mavlink_interface->SendGpsMessages(_gps_sensor->getData());
  }

  // Receive and handle actuator controls
  _mavlink_interface->pollForMAVLinkMessages();
  Eigen::VectorXd actuator_controls = _mavlink_interface->GetActuatorControls();

  if (actuator_controls.size() >= 16) {
    _actuators->SetActuatorCommands(actuator_controls);
  }

  result = _fdmexec->Run();

  std::chrono::duration<double> elapsed_time = current_time - _last_step_time;
  if (realtime) {
    double sleep = dt - elapsed_time.count();
    if (sleep > 0) usleep(sleep * 1e6);
  }

  _last_step_time = current_time;
}
