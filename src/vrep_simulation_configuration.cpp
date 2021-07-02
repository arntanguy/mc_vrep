#include "vrep_simulation_configuration.h"

#include <mc_rtc/version.h>

VREPSimulationConfiguration::VREPSimulationConfiguration(const mc_control::MCGlobalController & controller)
{
  auto vrep_c = controller.configuration().config("VREP", mc_rtc::Configuration{});
  vrep_c("Host", host);
  vrep_c("Port", port);
  vrep_c("Timeout", timeout);
  vrep_c("WaitUntilConnected", waitUntilConnected);
  vrep_c("DoNotReconnect", doNotReconnect);
  vrep_c("CommThreadCycleInMs", commThreadCycleInMs);
  vrep_c("SimulationTimestep", simulationTimestep);
  if(simulationTimestep < 0)
  {
    simulationTimestep = controller.timestep();
  }
  vrep_c("StepByStep", stepByStep);
  vrep_c("VelocityControl", velocityControl);
  vrep_c("TorqueControl", torqueControl);
  if(velocityControl && torqueControl)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Only one of VelocityControl or TorqueControl must be true");
  }
  if(vrep_c.has("Extras"))
  {
    auto extras_c = vrep_c("Extras");
    for(size_t i = 0; i < extras_c.size(); ++i)
    {
      if(extras_c[i].has("index"))
      {
        mc_rtc::log::warning("VREP::Extras is using an index to select an extra robot, this is deprecated");
        const auto & robots = controller.robots().robots();
        unsigned int idx = extras_c[i]("index");
        if(idx < robots.size())
        {
#if MC_RTC_VERSION_MAJOR < 2
          extras_c[i].add("name", robots[i].name());
#else
          extras_c[i].add("name", robots[i]->name());
#endif
        }
      }
      std::string name = extras_c[i]("name", std::string(""));
      std::string suffix = extras_c[i]("suffix", std::string(""));
      if(controller.robots().hasRobot(name))
      {
        extras.push_back({name, suffix});
      }
      else
      {
        mc_rtc::log::warning("Robot (named: \"{}\") in VREP extra configuration but not in controller", name);
      }
    }
  }
}
