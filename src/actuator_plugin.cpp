#include "actuator_plugin.h"
#include <tinyxml.h>

ActuatorPlugin::ActuatorPlugin(JSBSim::FGFDMExec *jsbsim) : sim_ptr_(jsbsim) {}

ActuatorPlugin::~ActuatorPlugin() {}

bool ActuatorPlugin::SetActuatorCommands(const Eigen::VectorXd &actuator_commands) {
  for (size_t i = 0; i < actuator_configs_.size(); i++) {
    size_t index = actuator_configs_[i].index;
    std::string property = actuator_configs_[i].property;
    double scale = actuator_configs_[i].scale;
    SetCommandToProperty(scale * actuator_commands[index], property);
  }
  return true;
}

bool ActuatorPlugin::SetCommandToProperty(float value, std::string property) {
  sim_ptr_->SetPropertyValue(property, value);
  return true;
}

bool ActuatorPlugin::SetActuatorConfigs(std::string &path) {
  TiXmlDocument doc(path);

  if (!doc.LoadFile()) return false;

  TiXmlHandle root(doc.RootElement());
  TiXmlElement *actuators = root.FirstChild("actuators").Element();

  for (TiXmlElement *e = actuators->FirstChildElement("channel"); e != NULL; e = e->NextSiblingElement("channel")) {
    ActuatorMap actuator_mapping;
    actuator_mapping.index = std::stoi(e->FirstChildElement("index")->GetText());
    actuator_mapping.scale = std::stoi(e->FirstChildElement("scale")->GetText());
    actuator_mapping.property = e->FirstChildElement("property")->GetText();
    actuator_configs_.push_back(actuator_mapping);
  }

  return true;
}
