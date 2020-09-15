
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
 * @file main.cpp
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 */

#include "jsbsim_bridge.h"

int main(int argc, char *argv[]) {
  if (argc < 5) {
    cout << "This is a JSBSim integration for PX4 SITL/HITL simulations" << std::endl;
    cout << "   Usage: " << argv[0] << "<aircraft_path> <aircraft> <config> <scene> <headless>" << endl;
    cout << "       <aircraft_path>: Aircraft directory path which the <aircraft> definition is located e.g. "
            "`models/Rascal`"
         << endl;
    cout << "       <aircraft>: Aircraft file to use inside the <aircraft_path> e.g. Rascal110-JSBSim" << endl;
    cout << "       <config>: Simulation config file name under the `configs` directory e.g. rascal" << endl;
    cout << "       <scene>: Location / scene where the vehicle should be spawned in e.g. LSZH" << endl;
    cout << "       <headless>: Headless option for flightgear visualiztion 1: enable 0: disable" << endl;
    return -1;
  }

  // Configure JSBSim
  JSBSim::FGFDMExec *fdmexec = new JSBSim::FGFDMExec();

  fdmexec->SetRootDir(SGPath(JSBSIM_ROOT_DIR));
  fdmexec->SetAircraftPath(SGPath(SGPath::fromLocal8Bit(argv[1])));
  fdmexec->SetEnginePath(SGPath("Engines"));

  bool headless = (argv[5] == "1");  // Check if HEADLESS mode is enabled
  if (!headless) {
    fdmexec->SetOutputDirectives(SGPath("data_out/flightgear.xml"));
  }

  // Load FDM model
  fdmexec->LoadModel(argv[2], false);

  // Load Initial Conditions
  JSBSim::FGInitialCondition *initial_condition = fdmexec->GetIC();
  SGPath init_script_path = SGPath::fromLocal8Bit(argv[4]);
  initial_condition->Load(SGPath(init_script_path), false);

  // Path to config file
  std::string path = std::string(JSBSIM_ROOT_DIR) + "/configs/" + std::string(argv[3]) + ".xml";

  std::unique_ptr<JSBSimBridge> jsbsim_bridge = std::make_unique<JSBSimBridge>(fdmexec, path);

  while (true) {
    jsbsim_bridge->Run();
  }
}
