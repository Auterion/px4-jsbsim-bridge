
/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 *
 * Mavlink HIL message interface to FlightGear and PX4
 */

#include "common.h"
#include "mavlink_interface.h"
#include "actuator_plugin.h"
#include "sensor_airspeed_plugin.h"
#include "sensor_baro_plugin.h"
#include "sensor_gps_plugin.h"
#include "sensor_imu_plugin.h"
#include "sensor_mag_plugin.h"

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>

#include <chrono>


int main(int argc, char *argv[]) {
  if (argc < 3) {
    cout << "This is a JSBSim integration for PX4 SITL/HITL simulations" << std::endl;
    cout << "   Usage: " << argv[0] << " <aircraft> <reset> <headless>" << endl;
    return -1;
  }

  double dt = 0.004;

  // Configure JSBSim
  JSBSim::FGFDMExec *fdmexec = new JSBSim::FGFDMExec();

  fdmexec->SetRootDir(SGPath(JSBSIM_ROOT_DIR));
  fdmexec->SetAircraftPath(SGPath(SGPath::fromLocal8Bit(argv[1])));
  fdmexec->SetEnginePath(SGPath("Engines"));
  bool headless = bool(argv[5]);
  if (!headless) {
    fdmexec->SetOutputDirectives(SGPath("data_out/flightgear.xml"));
  }

  fdmexec->LoadModel(argv[2], false);
  fdmexec->Setdt(dt);

  JSBSim::FGInitialCondition *initial_condition = fdmexec->GetIC();

  SGPath init_script_path = SGPath::fromLocal8Bit(argv[4]);
  initial_condition->Load(SGPath(init_script_path), false);

  fdmexec->RunIC();

  // Configure Mavlink HIL interface
  std::unique_ptr<MavlinkInterface> mavlink_interface_ = std::make_unique<MavlinkInterface>();

  mavlink_interface_->SetUseTcp(true);
  mavlink_interface_->SetEnableLockstep(true);
  mavlink_interface_->SetMavlinkTcpPort(4560);

  mavlink_interface_->Load();

  // Instantiate sensors
  std::unique_ptr<SensorImuPlugin> imu_sensor_ = std::make_unique<SensorImuPlugin>(fdmexec);
  std::unique_ptr<SensorGpsPlugin> gps_sensor_ = std::make_unique<SensorGpsPlugin>(fdmexec);
  gps_sensor_->setUpdateRate(1.0);
  std::unique_ptr<SensorBaroPlugin> baro_sensor_ = std::make_unique<SensorBaroPlugin>(fdmexec);
  std::unique_ptr<SensorMagPlugin> mag_sensor_ = std::make_unique<SensorMagPlugin>(fdmexec);
  std::unique_ptr<SensorAirspeedPlugin> airspeed_sensor_ = std::make_unique<SensorAirspeedPlugin>(fdmexec);

  std::unique_ptr<ActuatorPlugin> actuators_ = std::make_unique<ActuatorPlugin>(fdmexec);
  std::string path = std::string(JSBSIM_ROOT_DIR) + "/models/" + std::string(argv[3]) + ".xml";
  if (!actuators_->SetActuatorConfigs(path)) {
    std::cerr << "Could not load configuration file: " << path << std::endl;
    return 1;
  }

  bool result = true;
  bool realtime = true;

  auto last_step_time = std::chrono::system_clock::now();

  while (result) {
    // Get Simulation time from JSBSim
    auto current_time = std::chrono::system_clock::now();
    double simtime = fdmexec->GetSimTime();

    // Update sensor messages
    if (imu_sensor_->updated()) {
      // Only send sensor messages when the imu sensor is updated.
      // This is needed for lockstep
      mavlink_interface_->UpdateIMU(imu_sensor_->getData());

      if (mag_sensor_->updated()) mavlink_interface_->UpdateMag(mag_sensor_->getData());
      if (baro_sensor_->updated()) mavlink_interface_->UpdateBarometer(baro_sensor_->getData());
      if (airspeed_sensor_->updated()) mavlink_interface_->UpdateAirspeed(airspeed_sensor_->getData());

      // Send Mavlink HIL_SENSOR message
      mavlink_interface_->SendSensorMessages(simtime * 1e6);
    }

    // Send Mavlink HIL_GPS message
    if (gps_sensor_->updated()) mavlink_interface_->SendGpsMessages(gps_sensor_->getData());

    // Receive and handle actuator controls
    mavlink_interface_->pollForMAVLinkMessages();
    Eigen::VectorXd actuator_controls = mavlink_interface_->GetActuatorControls();

    if (actuator_controls.size() >= 16) {
      actuators_->SetActuatorCommands(actuator_controls);

    }

    result = fdmexec->Run();

    std::chrono::duration<double> elapsed_time = current_time - last_step_time;
    if (realtime) {
      double sleep = dt - elapsed_time.count();
      if (sleep > 0) usleep(sleep * 1e6);
    }

    last_step_time = current_time;
  }
}
