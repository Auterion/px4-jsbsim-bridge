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
 * @brief JSBSim Bridge Configuration Parser
 *
 * This is a class for the JSBSim actuator plugin
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 */

#include "configuration_parser.h"

ConfigurationParser::ConfigurationParser() {}

ConfigurationParser::~ConfigurationParser() {}

bool ConfigurationParser::ParseEnvironmentVariables() {
  if (const char* headless_char = std::getenv("HEADLESS")) {
    headless = !std::strcmp(headless_char, "1");
  }
  return true;
}

bool ConfigurationParser::ParseArgV(int argc, char* const argv[]) {
  // TODO: Parse HITL Variables
  if (argc < 5) {
    std::cout << "This is a JSBSim integration for PX4 SITL/HITL simulations" << std::endl;
    std::cout << "   Usage: " << argv[0] << "<aircraft_path> <aircraft> <config> <scene> <headless>" << std::endl;
    std::cout << "       <aircraft_path>: Aircraft directory path which the <aircraft> definition is located e.g. "
                 "`models/Rascal`"
              << std::endl;
    std::cout << "       <aircraft>: Aircraft file to use inside the <aircraft_path> e.g. Rascal110-JSBSim"
              << std::endl;
    std::cout << "       <config>: Simulation config file name under the `configs` directory e.g. rascal" << std::endl;
    std::cout << "       <scene>: Location / scene where the vehicle should be spawned in e.g. LSZH" << std::endl;
    std::cout << "       <headless>: Headless option for flightgear visualiztion 1: enable 0: disable" << std::endl;
    return false;
  }

  //TODO: Switch to getopt
  _init_script_path = std::string(argv[4]);

  return true;
}

bool ConfigurationParser::ParseConfigFile(const std::string& path) {
  doc = TiXmlDocument(path);
  if (!doc.LoadFile()) {
    std::cerr << "[ConfigurationParser] Could not load actuator configs from configuration file: " << path << std::endl;
    return false;
  }
  _config = new TiXmlHandle(doc.RootElement());

  TiXmlElement *model_config = _config->Element();
  if(model_config) {
    _model_name = model_config->Attribute("name");
  } else {
    std::cerr << "[ConfigurationParser] Incorrect or invalid model name" << std::endl;
    return false;    
  }

  return true;
}

bool ConfigurationParser::isHeadless() { return headless; }

TiXmlHandle* ConfigurationParser::LoadXmlHandle() {
  return _config;
}

std::string ConfigurationParser::getInitScriptPath() {
  return _init_script_path;
}

std::string ConfigurationParser::getModelName() {
  return _model_name;
}
