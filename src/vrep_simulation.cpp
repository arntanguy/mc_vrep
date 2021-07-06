/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "vrep_simulation.h"

#include <vrep-api-wrapper/vrep.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <array>
#include <fstream>
#include <iostream>
#include <map>

struct VREPSimulationImpl
{
private:
  mc_control::MCGlobalController & controller;
  vrep::VREP vrep;
  VREPCLI cli_;

  const bool velocityControl;

  const bool torqueControl;

  std::vector<VREPSimulationConfiguration::ExtraRobot> extraRobots;

  std::vector<std::string> rNames;
  std::vector<std::string> suffixes;
  std::vector<std::string> baseNames;
  std::vector<std::string> joints;
  std::vector<double> jQs;
  std::vector<double> jTorques;
  std::map<std::string, vrep::VREP::ForceSensor> fSensors;
  vrep::VREP::Accelerometer accel;
  vrep::VREP::Gyrometer gyro;
  std::vector<sva::PTransformd> basePoses;
  std::vector<sva::MotionVecd> baseVels;

  std::map<std::string, sva::ForceVecd> external_force;
  std::map<std::string, sva::ForceVecd> impact_force;

  double simulationTimestep = 0.005;
  size_t iter = 0;
  size_t frameskip = 1;

public:
  VREPSimulationImpl(VREPSimulation & self,
                     mc_control::MCGlobalController & controller,
                     const VREPSimulationConfiguration & c)
  : controller(controller),
    vrep(c.host, c.port, c.timeout, c.waitUntilConnected, c.doNotReconnect, c.commThreadCycleInMs),
    cli_(controller, self, c.stepByStep), velocityControl(c.velocityControl), torqueControl(c.torqueControl),
    extraRobots(c.extras)
  {
    auto & ctl = controller.controller();
#if MC_RTC_VERSION_MAJOR >= 2
    auto gui = &ctl.gui();
#else
    auto gui = ctl.gui();
#endif
    if(gui)
    {
      auto data = gui->data();
      auto vrep_data = data.add("VREP");
      auto vrep_bodies = vrep_data.array("bodies");
      for(auto & b : ctl.robot().mb().bodies())
      {
        if(b.inertia().mass() != 0)
        {
          vrep_bodies.push(b.name() + "_respondable");
        }
      }
      gui->addElement({"VREP", "Force"},
                      mc_rtc::gui::Form(
                          "Apply force",
                          [this, gui](const mc_rtc::Configuration & data) {
                            std::string body = data("Body");
                            Eigen::Vector6d force = data("Force");
                            setExternalForce(body, {force});
                            gui->addElement({"VREP", "Force"},
                                            mc_rtc::gui::Button("Remove force on " + body, [this, body, gui]() {
                                              std::string msg = "removeForce " + body;
                                              removeExternalForce(body);
                                              gui->removeElement({"VREP", "Force"}, "Remove force on " + body);
                                            }));
                          },
                          mc_rtc::gui::FormDataComboInput("Body", true, {"VREP", "bodies"}),
                          mc_rtc::gui::FormArrayInput<Eigen::Vector6d>("Force", true, Eigen::Vector6d::Zero())));
      gui->addElement({"VREP", "Impact"},
                      mc_rtc::gui::Form(
                          "Apply impact",
                          [this, gui](const mc_rtc::Configuration & data) {
                            std::string body = data("Body");
                            Eigen::Vector6d force = data("Force (N.s)");
                            applyImpact(body, {force});
                          },
                          mc_rtc::gui::FormDataComboInput("Body", true, {"VREP", "bodies"}),
                          mc_rtc::gui::FormArrayInput<Eigen::Vector6d>("Force (N.s)", true, Eigen::Vector6d::Zero())));
      gui->addElement(
          {"VREP"},
          mc_rtc::gui::Checkbox(
              "Step by step", [this]() { return cli_.stepByStep(); }, [this]() { cli_.toggleStepByStep(); }),
          mc_rtc::gui::Button("Next step", [this]() { cli_.nextStep(); }), mc_rtc::gui::Button("Stop", [this]() {
            stopSimulation();
            std::exit(0);
          }));
    }

    this->simulationTimestep = c.simulationTimestep;
#if MC_RTC_VERSION_MAJOR < 2
    frameskip = std::round(ctl.timeStep / simulationTimestep);
#else
    frameskip = std::round(ctl.solver().dt() / simulationTimestep);
#endif
    mc_rtc::log::info("[mc_vrep] Frameskip: {}", frameskip);
  }

  typedef sva::ForceVecd wrench_t;
  std::map<std::string, wrench_t> wrenches(const mc_rbdyn::Robot & robot, const std::string & suffix)
  {
    std::map<std::string, wrench_t> res;
    for(const auto & fs : robot.forceSensors())
    {
      for(const auto & fIn : fSensors)
      {
        if(fIn.first == fs.name() + suffix)
        {
          res.emplace(fs.name(), wrench_t{fIn.second.torque, fIn.second.force});
        }
      }
    }
    return res;
  }

  void startSimulation()
  {
    auto & robots = controller.controller().robots();
    for(size_t i = controller.realRobots().size(); i < robots.size(); ++i)
    {
#if MC_RTC_VERSION_MAJOR < 2
      const auto & robot = robots.robot(i);
#else
      const auto & robot = *robots.robots()[i];
#endif
      controller.realRobots().robotCopy(robot, robot.name());
    }
    rNames.push_back(controller.robot().name());
    suffixes.push_back("");
    for(auto & e : extraRobots)
    {
      rNames.push_back(e.name);
      suffixes.push_back(e.suffix);
    }
    for(size_t i = 0; i < rNames.size(); ++i)
    {
      const auto & suffix = suffixes[i];
      const auto & robot = robots.robot(rNames[i]);
      controller.realRobots().robotCopy(robot, robot.name());
      std::string jName = "";
      std::string baseName = jName;
      for(const auto & j : robot.mb().joints())
      {
        if(j.dof() == 1)
        {
          jName = j.name();
          break;
        }
      }
      if(jName == "" && i == 0)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("No 1-dof joints in your main robot");
      }
      else if(jName == "")
      {
        if(robot.mb().bodies().size() > 1 && robot.mb().body(0).name() == "base_link")
        {
          baseName = robot.mb().body(1).name();
        }
        else
        {
          baseName = robot.mb().body(0).name();
        }
        mc_rtc::log::warning("ExtraRobot with index {} cannot be controlled, will only track the base position {}", i,
                             baseName);
      }
      else
      {
        baseName = vrep.getModelBase(jName + suffix);
      }
      baseNames.push_back(baseName);
      for(const auto & fs : robot.forceSensors())
      {
        fSensors[fs.name() + suffix] = {};
      }
      for(const auto & j : robot.refJointOrder())
      {
        joints.push_back(j + suffix);
      }
    }
    vrep.startSimulation(baseNames, joints, fSensors);
    /* Run simulation until the data arrives */
    while(!vrep.getSimulationState(joints, jQs, jTorques, fSensors, accel, gyro, baseNames, basePoses, baseVels))
    {
      vrep.nextSimulationStep();
    }
    for(size_t i = 0; i < 10; ++i)
    {
      vrep.nextSimulationStep();
    }
    controller.running = true;
    for(size_t i = 0; i < rNames.size(); ++i)
    {
      auto & robot = robots.robot(rNames[i]);
      robot.posW(basePoses[i]);
    }
    updateData();
    controller.init(controller.robot().encoderValues());
    mc_rtc::log::success("Simulation started");
  }

  void updateData()
  {
    size_t jQi = 0;
    for(size_t i = 0; i < rNames.size(); ++i)
    {
      auto & robot = controller.controller().robots().robot(rNames[i]);
      controller.setSensorPosition(robot.name(), basePoses[i].translation());
      controller.setSensorOrientation(robot.name(), Eigen::Quaterniond(basePoses[i].rotation()));
      controller.setSensorLinearVelocity(robot.name(), baseVels[i].linear());
      controller.setSensorAngularVelocity(robot.name(), baseVels[i].angular());
      std::vector<double> encoders(robot.refJointOrder().size());
      std::vector<double> torques(robot.refJointOrder().size());
      for(size_t j = 0; j < robot.refJointOrder().size(); ++j)
      {
        encoders[j] = jQs[jQi + j];
        torques[j] = jTorques[jQi + j];
      }
      jQi += robot.refJointOrder().size();
      controller.setEncoderValues(robot.name(), encoders);
      controller.setJointTorques(robot.name(), jTorques);
      controller.setWrenches(robot.name(), wrenches(robot, suffixes[i]));
    }
    controller.setSensorLinearAcceleration(accel.data);
  }

  bool setExternalForce(const std::string & body_respondable, const sva::ForceVecd & force)
  {
    external_force[body_respondable] = force;
    return true;
  }

  bool removeExternalForce(const std::string & body_respondable)
  {
    if(external_force.count(body_respondable))
    {
      external_force.erase(body_respondable);
      return true;
    }
    return false;
  }

  bool applyImpact(const std::string & body_respondable, const sva::ForceVecd & impact)
  {
    impact_force[body_respondable] = impact / controller.timestep();
    return true;
  }

  void nextSimulationStep()
  {
    float startT = vrep.getSimulationTime();
    static float prevT = startT - simulationTimestep;
    if(fabs(startT - prevT - simulationTimestep) > 1e-4)
    {
      std::cerr << "Missed a simulation step " << startT << " " << prevT << "\n";
    }
    prevT = startT;
    if(iter % frameskip == 0)
    {
      vrep.getSimulationState(joints, jQs, jTorques, fSensors, accel, gyro, baseNames, basePoses, baseVels);

      // Add external forces
      for(const auto & f : external_force)
      {
        vrep.addForce(f.first, f.second);
      }

      // apply impact forces
      for(const auto & f : impact_force)
      {
        vrep.addForce(f.first, f.second);
      }
      impact_force.clear();

      updateData();
      if(controller.run())
      {
        for(size_t i = 0; i < rNames.size(); ++i)
        {
          auto & robot = controller.controller().robots().robot(rNames[i]);
          const auto & suffix = suffixes[i];
          if(torqueControl)
          {
            vrep.setRobotTargetTorque(robot.mb(), robot.mbc(), suffix);
          }
          else if(velocityControl)
          {
            vrep.setRobotTargetVelocity(robot.mb(), robot.mbc(), suffix);
          }
          else
          {
            vrep.setRobotTargetConfiguration(robot.mb(), robot.mbc(), suffix);
          }
        }
      }
    }
    iter++;
    float endT = vrep.getSimulationTime();
    if(endT != startT)
    {
      std::cerr << "One iteration occured while the simulation was running\n";
    }
    vrep.nextSimulationStep();
  }

  void stopSimulation()
  {
    vrep.stopSimulation();
  }

  void updateGUI()
  {
    controller.running = false;
    controller.run();
    controller.running = true;
  }

  VREPCLI & cli()
  {
    return cli_;
  }
};

VREPSimulation::VREPSimulation(mc_control::MCGlobalController & controller, const VREPSimulationConfiguration & config)
: impl(new VREPSimulationImpl(*this, controller, config))
{
}

VREPSimulation::~VREPSimulation() {}

void VREPSimulation::startSimulation()
{
  impl->startSimulation();
}

void VREPSimulation::nextSimulationStep()
{
  impl->nextSimulationStep();
}

void VREPSimulation::stopSimulation()
{
  impl->stopSimulation();
}

void VREPSimulation::updateGUI()
{
  impl->updateGUI();
}

bool VREPSimulation::setExternalForce(const std::string & body_respondable, const sva::ForceVecd & force)
{
  return impl->setExternalForce(body_respondable, force);
}

bool VREPSimulation::removeExternalForce(const std::string & body_respondable)
{
  return impl->removeExternalForce(body_respondable);
}

bool VREPSimulation::applyImpact(const std::string & body_respondable, const sva::ForceVecd & impact)
{
  return impl->applyImpact(body_respondable, impact);
}

VREPCLI & VREPSimulation::cli()
{
  return impl->cli();
}
