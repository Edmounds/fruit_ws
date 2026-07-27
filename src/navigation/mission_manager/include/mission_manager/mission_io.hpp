#ifndef MISSION_MANAGER__MISSION_IO_HPP_
#define MISSION_MANAGER__MISSION_IO_HPP_

#include <string>

#include "mission_manager/mission_types.hpp"

namespace mission_manager {

bool load_mission_file(const std::string& mission_path,
                       MissionDefinition& mission,
                       std::string& error_message);

bool mission_from_yaml_string(const std::string& mission_yaml,
                              MissionDefinition& mission,
                              std::string& error_message);

bool save_mission_file(const std::string& mission_path,
                       const MissionDefinition& mission,
                       std::string& error_message);

std::string mission_to_yaml_string(const MissionDefinition& mission);

}  // namespace mission_manager

#endif  // MISSION_MANAGER__MISSION_IO_HPP_
