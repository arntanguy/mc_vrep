#pragma once

#include <mc_control/mc_global_controller.h>

/** Configuration for the connection to VREP and the simulation */
struct VREPSimulationConfiguration
{
  /** See vrep::VREP documentation */
  std::string host = "127.0.0.1";
  /** See vrep::VREP documentation */
  int port = 19997;
  /** See vrep::VREP documentation */
  int timeout = 3000;
  /** See vrep::VREP documentation */
  bool waitUntilConnected = true;
  /** See vrep::VREP documentation */
  bool doNotReconnect = true;
  /** See vrep::VREP documentation */
  int commThreadCycleInMs = 1;
  /** Simulation timestep, defaults to the controller timestep if not provided */
  double simulationTimestep = -1;
  /** If true, start the simulation step by step */
  bool stepByStep = false;
  /** If true, use joint velocity rather than joint position */
  bool velocityControl = false;
  /** If true, use computed torques as control input rather than joint position */
  bool torqueControl = false;
  /** Configuration for extra-robots */
  struct ExtraRobot
  {
    std::string name;
    std::string suffix;
  };
  /** Suffix to apply given a robot index */
  std::vector<ExtraRobot> extras = {};

  VREPSimulationConfiguration() = default;
  VREPSimulationConfiguration(const mc_control::MCGlobalController & controller);
};
