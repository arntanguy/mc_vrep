/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/config.h>

#include "vrep_cli.h"
#include "vrep_simulation_configuration.h"

struct VREPSimulationImpl;

/*! \brief This class is a thin wrapper around the
 * vrep-api-wrapper to drive a simulation in VREP using the
 * mc_rtc control framework.
 */
struct VREPSimulation
{
public:
  /*! \brief Constructor
   *
   * Prepare to start a simulation.
   *
   * \param controller The mc_rtc controller instance used in the simulation.
   * \param config See VREPSimulationConfiguration
   *
   */
  VREPSimulation(mc_control::MCGlobalController & controller, const VREPSimulationConfiguration & config);

  /*! \brief Destructor */
  ~VREPSimulation();

  /*! \brief Start the simulation. This should be called
   * only once
   */
  void startSimulation();

  /*! Trigger the next simulation step. This should be
   * called as long as the simulation is running.
   */
  void nextSimulationStep();

  /*! Stop the simulation */
  void stopSimulation();

  /*! Update the GUI */
  void updateGUI();

  /*! Apply an external force to a respondable body. The force will be applied at
   * each iteration until it is explicitely removed.
   *
   * \param body_respondable Name of respondable body in V-REP scene
   *
   * \param force External force in world frame
   *
   */
  bool setExternalForce(const std::string & body_respondable, const sva::ForceVecd & force);

  /*! Remove external force applied on a respondable body.
   *
   * \param body_respondable Name of respondable body in V-REP scene
   *
   */
  bool removeExternalForce(const std::string & body_respondable);

  /*! Add an external impact force. The force will be applied for one iteration only,
   * then removed immediately.
   *
   * \param body_respondable Name of respondable body in V-REP scene
   *
   * \param impact Impact vector in world frame (impact = force * dt where dt is the
   * simulation time step)
   *
   */
  bool applyImpact(const std::string & body_respondable, const sva::ForceVecd & impact);

  /*! Access the CLI associated to this simulation instance */
  VREPCLI & cli();

private:
  std::unique_ptr<VREPSimulationImpl> impl;
};
