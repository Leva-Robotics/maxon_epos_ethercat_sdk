// clang-format off
/*
** Copyright 2021 Robotic Systems Lab - ETH Zurich:
** Linghao Zhang, Jonas Junger, Lennart Nachtigall
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
** 1. Redistributions of source code must retain the above copyright notice,
**    this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder nor the names of its contributors
**    may be used to endorse or promote products derived from this software without
**    specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// clang-format on

#include <cstdint>

#include "maxon_epos_ethercat_sdk/ConfigurationParser.hpp"

namespace maxon {
/*!
 * Function template for convenience
 * @param[in] yamlNode	the current node containing the requested variable
 * @param[in] varName	The name of the variable
 * @param[out] var	The variable which shall be read
 * @return	true on success
 */
template <typename T>
bool getValueFromFile(YAML::Node& yamlNode, const std::string& varName,
                      T& var) {
  if (!yamlNode[varName].IsDefined()) {
    MELO_WARN_STREAM(
        "[maxon_epos_ethercat_sdk:ConfigurationParser::parseConfiguration]: "
        "field '"
        << varName << "' is missing. Default value will be used.");
    return false;
  }
  try {
    T tmpVar = yamlNode[varName].as<T>();
    var = tmpVar;
    return true;
  } catch (...) {
    MELO_ERROR_STREAM(
        "[maxon_epos_ethercat_sdk:ConfigurationParser::getValueFromFile] Error "
        "while parsing value \""
        << varName << "\", default values will be used");
    return false;
  }
}

/*!
 * Function to read a Modes of Operation enum from the yaml file
 * @param[in] yamlNode	the node containing the requested value
 * @param [in] varName	The name of the variable
 * @param [out] mode	The read mode of operation
 * @return	true on success
 */
bool getModesFromFile(YAML::Node& yamlNode, const std::string& varName,
                      std::vector<ModeOfOperationEnum>& modes) {
  if (!yamlNode[varName].IsDefined()) {
    MELO_WARN_STREAM(
        "[maxon_epos_ethercat_sdk:ConfigurationParser::parseConfiguration]: "
        "field '"
        << varName << "' is missing. Default value will be used.");
    return false;
  }
  try {
    const std::map<std::string, ModeOfOperationEnum> str2ModeMap = {
        {"ProfiledVelocityMode", ModeOfOperationEnum::ProfiledVelocityMode},
        {"CyclicSynchronousTorqueMode",ModeOfOperationEnum::CyclicSynchronousTorqueMode},
        {"CyclicJVPTMode", ModeOfOperationEnum::CyclicJVPTMode},
        {"CyclicFreezeMode", ModeOfOperationEnum::CyclicFreezeMode},
        {"HomingMode", ModeOfOperationEnum::HomingMode},
    };

    std::vector<std::string> strModes =
        yamlNode[varName].as<std::vector<std::string>>();
    for (const auto& strMode : strModes) {
      if (str2ModeMap.find(strMode) != str2ModeMap.end()) {
        modes.push_back(str2ModeMap.at(strMode));
      } else {
        MELO_ERROR_STREAM(
            "[maxon_epos_ethercat_sdk:ConfigurationParser::parseConfiguration]"
            << "Mode '" << strMode << "' Does not exist.")
        return false;
      }
    }
    return true;
  } catch (...) {
    MELO_ERROR_STREAM(
        "[maxon_epos_ethercat_sdk:ConfigurationParser::getModeFromFile] Error "
        "while parsing value \""
        << varName << "\", default values will be used");
    return false;
  }
}

double getVelocityFactorConfiguredUnitToRadPerSecFromFile(uint32_t velConfig){
  switch (velConfig) {
    case 0x00B44700: //rpm
      return 2.0 * M_PI / (60.0);
    case 0xFFB44700: //dezi rpm
      return (1.0/10.0) * 2.0*M_PI/(60.0);
    case 0xFEB44700: //centi rpm
      return (1.0/100.0) * 2.0*M_PI/(60.0);
    case 0xFDB44700: //milli rpm
      return (1.0/1000.0) * 2.0*M_PI/(60.0);
    case 0xFCB44700: //10-e4
      return (1.0/10000.0) * 2.0*M_PI/(60.0);
    case 0xFBB44700: //10-e5
      return (1.0/100000.0) * 2.0*M_PI/(60.0);
    case 0xFAB44700:
      MELO_WARN_STREAM("[MaxonSDK] Velocity unit configured in microRPM: max velocity limit by max int to ~210 rad/s")
      return (1.0/1000000.0) * 2.0*M_PI/(60.0);
    default:
      MELO_ERROR_STREAM("[MaxonSDK] Could not read velocity unit configuration")
      return 0;
  }
}

ConfigurationParser::ConfigurationParser(const std::string& filename) {
  YAML::Node configNode;
  try {
    configNode = YAML::LoadFile(filename);
  } catch (...) {
    MELO_FATAL_STREAM(
        "[maxon_epos_ethercat_sdk:ConfigurationParser::ConfigurationParser] "
        "Loading YAML configuration file '"
        << filename << "' failed.");
  }
  parseConfiguration(configNode);
}

ConfigurationParser::ConfigurationParser(YAML::Node configNode) {
  parseConfiguration(configNode);
}

void ConfigurationParser::parseConfiguration(YAML::Node configNode) {
  if (configNode["Maxon"].IsDefined()) {
    /// A new node for the MaxonEthercat class
    YAML::Node maxonNode = configNode["Maxon"];

    unsigned int configRunSdoVerifyTimeout;
    if (getValueFromFile(maxonNode, "config_run_sdo_verify_timeout",
                         configRunSdoVerifyTimeout)) {
      configuration_.configRunSdoVerifyTimeout = configRunSdoVerifyTimeout;
    }

    bool printDebugMessages;
    if (getValueFromFile(maxonNode, "print_debug_messages",
                         printDebugMessages)) {
      configuration_.printDebugMessages = printDebugMessages;
    }

    bool useRawCommands;
    if (getValueFromFile(maxonNode, "use_raw_commands", useRawCommands)) {
      configuration_.useRawCommands = useRawCommands;
    }

    unsigned int driveStateChangeMinTimeout;
    if (getValueFromFile(maxonNode, "drive_state_change_min_timeout",
                         driveStateChangeMinTimeout)) {
      configuration_.driveStateChangeMinTimeout = driveStateChangeMinTimeout;
    }

    unsigned int minNumberOfSuccessfulTargetStateReadings;
    if (getValueFromFile(maxonNode,
                         "min_number_of_successful_target_state_readings",
                         minNumberOfSuccessfulTargetStateReadings)) {
      configuration_.minNumberOfSuccessfulTargetStateReadings =
          minNumberOfSuccessfulTargetStateReadings;
    }

    unsigned int driveStateChangeMaxTimeout;
    if (getValueFromFile(maxonNode, "drive_state_change_max_timeout",
                         driveStateChangeMaxTimeout)) {
      configuration_.driveStateChangeMaxTimeout = driveStateChangeMaxTimeout;
    }
  }

  /// The configuration options for the maxon::ethercat::Reading class
  if (configNode["Reading"].IsDefined()) {
    YAML::Node readingNode = configNode["Reading"];

    bool forceAppendEqualError;
    if (getValueFromFile(readingNode, "force_append_equal_error",
                         forceAppendEqualError)) {
      configuration_.forceAppendEqualError = forceAppendEqualError;
    }

    bool forceAppendEqualFault;
    if (getValueFromFile(readingNode, "force_append_equal_fault",
                         forceAppendEqualFault)) {
      configuration_.forceAppendEqualFault = forceAppendEqualFault;
    }

    unsigned int errorStorageCapacity;
    if (getValueFromFile(readingNode, "error_storage_capacity",
                         errorStorageCapacity)) {
      configuration_.errorStorageCapacity = errorStorageCapacity;
    }

    unsigned int faultStorageCapacity;
    if (getValueFromFile(readingNode, "fault_storage_capacity",
                         faultStorageCapacity)) {
      configuration_.faultStorageCapacity = faultStorageCapacity;
    }
  }

  /// The configuration options for the Maxon servo drive ("hardware")
  if (configNode["Hardware"].IsDefined()) {
    YAML::Node hardwareNode = configNode["Hardware"];

    std::vector<ModeOfOperationEnum> modesOfOperation;
    if (getModesFromFile(hardwareNode, "mode_of_operation", modesOfOperation)) {
      configuration_.modesOfOperation = modesOfOperation;
    }

    int32_t positionEncoderResolution;
    if (getValueFromFile(hardwareNode, "position_encoder_resolution",
                         positionEncoderResolution)) {
      configuration_.positionEncoderResolution = positionEncoderResolution;
    }

    std::pair<float, float> gearRatio;
    if (getValueFromFile(hardwareNode, "gear_ratio", gearRatio)) {
      configuration_.gearRatio = static_cast<double>(gearRatio.first) /
                                 static_cast<double>(gearRatio.second);
    }


    //Anydrive5 specific settings

    double maxTorqueSI;
    if (getValueFromFile(hardwareNode, "max_torque", maxTorqueSI)) {
      configuration_.maxTorqueSI = maxTorqueSI;
    }

    double jvptPGain;
    if (getValueFromFile(hardwareNode, "JVPT_P_gain", jvptPGain)) {
      configuration_.jvptPGain = jvptPGain;
    }

    double jvptIGain;
    if (getValueFromFile(hardwareNode, "JVPT_I_gain", jvptIGain)) {
      configuration_.jvptIGain = jvptIGain;
    }

    double jvptDGain;
    if (getValueFromFile(hardwareNode, "JVPT_D_gain", jvptDGain)) {
      configuration_.jvptDGain = jvptDGain;
    }

    double softMaxPosLimitSI;
    if (getValueFromFile(hardwareNode, "soft_max_pos_limit", softMaxPosLimitSI)) {
      configuration_.softMaxPosLimitSI = softMaxPosLimitSI;
    }

    double softMinPosLimitSI;
    if (getValueFromFile(hardwareNode, "soft_min_pos_limit", softMinPosLimitSI)) {
      configuration_.softMinPosLimitSI = softMinPosLimitSI;
    }

  }
}

Configuration ConfigurationParser::getConfiguration() const {
  return configuration_;
}

}  // namespace maxon
