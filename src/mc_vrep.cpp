/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "vrep_simulation.h"

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <cmath>
#include <iostream>
#include <thread>

void simThread(VREPSimulation & vrep)
{
  auto & cli = vrep.cli();
  while(!cli.done())
  {
    vrep.nextSimulationStep();
    while(cli.stepByStep() && !cli.next() && !cli.done())
    {
      vrep.updateGUI();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    cli.play();
  }
  vrep.stopSimulation();
}

int main(int argc, char * argv[])
{
  /* Create a global controller */
  std::string conf_file = "";
  if(argc > 1)
  {
    conf_file = argv[1];
  }

  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error("mc_vrep was compiled with {} but mc_rtc is at version {}, you might "
                       "face subtle issues or unexpected crashes, please recompile mc_vrep",
                       mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }

  mc_control::MCGlobalController controller(conf_file);

  /* Start the VREP remote API */
  VREPSimulationConfiguration config(controller);
  auto vrep_c = controller.configuration().config("VREP", mc_rtc::Configuration{});
  VREPSimulation vrep(controller, config);

  vrep.startSimulation();

  auto & cli = vrep.cli();
  std::thread th(std::bind(&VREPCLI::run, &cli));
  simThread(vrep);

  th.join();
  return 0;
}
