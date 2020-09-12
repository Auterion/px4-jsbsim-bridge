
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


JSBSimBridge::JSBSimBridge(JSBSim::FGFDMExec *fdmexec, std::string &path) :
  fdmexec_(fdmexec),
  realtime(true),
  result(true),
  dt(0.004) {

  TiXmlDocument doc(path);  
  if (!doc.LoadFile()){
    std::cerr << "Could not load actuator configs from configuration file: " << path << std::endl;
    return;
  }
  TiXmlHandle config(doc.RootElement());

  fdmexec_->RunIC();

  // Configure Mavlink HIL interface
  mavlink_interface_ = std::make_unique<MavlinkInterface>();
  SetMavlinkInterfaceConfigs(mavlink_interface_, config);

  mavlink_interface_->Load();

  //TODO: Only instantiate sensors that are in the config file
  // Instantiate sensors
  imu_sensor_ = std::make_unique<SensorImuPlugin>(fdmexec_);
  gps_sensor_ = std::make_unique<SensorGpsPlugin>(fdmexec_);
  gps_sensor_->setUpdateRate(1.0);
  baro_sensor_ = std::make_unique<SensorBaroPlugin>(fdmexec_);
  mag_sensor_ = std::make_unique<SensorMagPlugin>(fdmexec_);
  airspeed_sensor_ = std::make_unique<SensorAirspeedPlugin>(fdmexec_);

  actuators_ = std::make_unique<ActuatorPlugin>(fdmexec_);
  actuators_->SetActuatorConfigs(config);

  last_step_time = std::chrono::system_clock::now();
}

JSBSimBridge::~JSBSimBridge() {

}

void JSBSimBridge::Run() {
  worker = std::thread(&JSBSimBridge::Thread, this);
  worker.join();
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

void JSBSimBridge::Thread() {
  while (true) {
    // Get Simulation time from JSBSim
    auto current_time = std::chrono::system_clock::now();
    double simtime = fdmexec_->GetSimTime();

    // Update sensor messages
    if (imu_sensor_) {
      if (imu_sensor_->updated()) {
        // Only send sensor messages when the imu sensor is updated.
        // This is needed for lockstep
        mavlink_interface_->UpdateIMU(imu_sensor_->getData());

        if (mag_sensor_) {
          if (mag_sensor_->updated()) mavlink_interface_->UpdateMag(mag_sensor_->getData());
        }

        if (baro_sensor_) {
          if (baro_sensor_->updated()) mavlink_interface_->UpdateBarometer(baro_sensor_->getData());
        }

        if (airspeed_sensor_) {
          if (airspeed_sensor_->updated()) mavlink_interface_->UpdateAirspeed(airspeed_sensor_->getData());
        }

        // Send Mavlink HIL_SENSOR message
        mavlink_interface_->SendSensorMessages(simtime * 1e6);
      }
    }

    // Send Mavlink HIL_GPS message
    if (gps_sensor_) {
      if (gps_sensor_->updated()) mavlink_interface_->SendGpsMessages(gps_sensor_->getData());
    }

    // Receive and handle actuator controls
    mavlink_interface_->pollForMAVLinkMessages();
    Eigen::VectorXd actuator_controls = mavlink_interface_->GetActuatorControls();

    if (actuator_controls.size() >= 16) {
      actuators_->SetActuatorCommands(actuator_controls);
    }

    result = fdmexec_->Run();

    std::chrono::duration<double> elapsed_time = current_time - last_step_time;
    if (realtime) {
      double sleep = dt - elapsed_time.count();
      if (sleep > 0) usleep(sleep * 1e6);
    }

    last_step_time = current_time;
  }
}
