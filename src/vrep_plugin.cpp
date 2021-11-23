#include "simLib.h"

static LIBRARY simLib; // the CoppelisSim library that we will dynamically load and bind

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

// For the configuration
#include "vrep_simulation_configuration.h"

// For logging
#include "LogSink.h"

#ifdef _WIN32
#  define SIM_DLLEXPORT extern "C" __declspec(dllexport)
#else
#  define SIM_DLLEXPORT extern "C"
#endif

#if defined(__linux) || defined(__APPLE__)
#  include <string.h>
#  include <unistd.h>
#  define _stricmp(x, y) strcasecmp(x, y)
#endif

#define PLUGIN_VERSION 1

namespace mc_vrep
{

static std::unique_ptr<mc_control::MCGlobalController::GlobalConfiguration> gc_config_ptr;
static std::unique_ptr<mc_control::ControllerServer> server_ptr;
static std::unique_ptr<mc_rtc::gui::StateBuilder> default_gui_ptr;
static std::unique_ptr<mc_control::MCGlobalController> gc_ptr;

/** True when the simulation is started from mc_rtc GUI */
static bool simulation_started = false;
/** True when the simulation is paused from mc_rtc GUI */
static bool simulation_paused = false;
/** True when step-by-step is enabled */
static bool simulation_step_by_step = false;
/** Number of steps left to run in step-by-step mode */
static size_t simulation_steps = 0;
/** True when the simulation is stopped from mc_rtc GUI */
static bool simulation_stopped = false;
/** True when the simulation is reset from mc_rtc GUI */
static bool simulation_reset = false;

/** Log sink for most messages */
static std::shared_ptr<LogSink> sink;
/** Log sink for success messages */
static std::shared_ptr<SuccessSink> success_sink;

namespace utils
{

bool isModelBase(int handle)
{
  auto prop = simGetModelProperty(handle);
  return (prop & sim_modelproperty_not_model) == 0;
}

int getHandleRoot(int handle)
{
  auto h = handle;
  while(simGetObjectParent(h) != -1)
  {
    h = simGetObjectParent(h);
  }
  return h;
}

int getModelBaseHandle(const std::string & name)
{
  auto h = simGetObjectHandle(name.c_str());
  while(h != -1 && !isModelBase(h))
  {
    h = simGetObjectParent(h);
  }
  return h;
}

sva::PTransformd getObjectPose(int handle)
{
  sva::PTransformd out;
  float pos[7];
  // pos is tx,ty,tz,qx,qy,qz,qw
  simGetObjectPose(handle, -1, pos);
  out.translation() << pos[0], pos[1], pos[2];
  Eigen::Quaterniond orientation;
  orientation.w() = pos[6];
  orientation.x() = pos[3];
  orientation.y() = pos[4];
  orientation.z() = pos[5];
  orientation = orientation.inverse();
  out.rotation() = orientation.toRotationMatrix();
  return out;
}

} // namespace utils

/** Data for the simulation */
struct SimulationData
{
  /** Constructor initialize data from the scene current state before the simulation starts */
  SimulationData(mc_control::MCGlobalController & controller);

  /** Run loop
   * - Update sensors from the scene
   * - Run mc_rtc controller
   * - Send the commmand to the robots
   */
  void run();

private:
  mc_control::MCGlobalController & gc_;
  VREPSimulationConfiguration config_;
  struct VREPRobot
  {
    struct VREPJoint
    {
      /* Joint handle (-1 if the reference joint is not in simulation) */
      int handle;
      /* True if the joint is a on negative axis (we then need to reverse the data to/from CoppeliaSim) */
      bool reversed;
    };

    VREPRobot() {}

    VREPRobot(std::string name,
              int rootHandle,
              const sva::PTransformd & X_0_vrep,
              const std::vector<VREPJoint> & refJointToHandle,
              const std::unordered_map<std::string, int> & fsHandles,
              const std::vector<std::string> & bodySensors)
    : name(name), rootHandle(rootHandle), X_0_vrep(X_0_vrep), refJointToHandle(refJointToHandle), fsHandles(fsHandles),
      bodySensors(bodySensors)
    {
      joints.resize(refJointToHandle.size());
      jointsVelocity.resize(refJointToHandle.size());
      jointsTorque.resize(refJointToHandle.size());
      for(auto & fs : fsHandles)
      {
        if(fs.second != -1)
        {
          wrenches[fs.first] = sva::ForceVecd::Zero();
        }
      }
      for(auto & bs : bodySensors)
      {
        angularVelocities[bs] = Eigen::Vector3d::Zero();
        linearAccelerations[bs] = Eigen::Vector3d::Zero();
      }
    }

    void updateSensors(mc_control::MCGlobalController & gc)
    {
      const auto & robot = gc.robot(name);
      const auto & rjo = robot.refJointOrder();
      if(robot.hasBodySensor("FloatingBase"))
      {
        sva::PTransformd X_0_r = utils::getObjectPose(rootHandle) * X_0_vrep;
        position = X_0_r.translation();
        orientation = X_0_r.rotation();
        gc.setSensorPositions(name, {{"FloatingBase", position}});
        gc.setSensorOrientations(name, {{"FloatingBase", orientation}});
      }
      {
        float linV[3];
        float accV[3];
        if(robot.hasBodySensor("FloatingBase"))
        {
          simGetObjectVelocity(rootHandle, linV, accV);
          linearVelocity << linV[0], linV[1], linV[2];
          angularVelocities["FloatingBase"] << accV[0], accV[1], accV[2];
          gc.setSensorLinearVelocities(name, {{"FloatingBase", linearVelocity}});
        }
        for(const auto & bs : bodySensors)
        {
          float x, y, z = 0.0f;
          simGetFloatSignal(fmt::format("{}_GyroSensor_x", bs).c_str(), &x);
          simGetFloatSignal(fmt::format("{}_GyroSensor_y", bs).c_str(), &y);
          simGetFloatSignal(fmt::format("{}_GyroSensor_z", bs).c_str(), &z);
          angularVelocities[bs] << x, y, z;
        }
        gc.setSensorAngularVelocities(name, angularVelocities);
      }
      for(const auto & bs : bodySensors)
      {
        float x, y, z = 0.0f;
        simGetFloatSignal(fmt::format("{}_x", bs).c_str(), &x);
        simGetFloatSignal(fmt::format("{}_y", bs).c_str(), &y);
        simGetFloatSignal(fmt::format("{}_z", bs).c_str(), &z);
        linearAccelerations[bs] << x, y, z;
      }
      gc.setSensorLinearAccelerations(name, linearAccelerations);
      for(size_t i = 0; i < rjo.size(); ++i)
      {
        const auto & j = rjo[i];
        if(refJointToHandle[i].handle != -1)
        {
          float data = 0;
          auto h = refJointToHandle[i].handle;
          double d = refJointToHandle[i].reversed ? -1 : 1;
          simGetJointPosition(h, &data);
          joints[i] = d * data;
          simGetJointForce(h, &data);
          jointsTorque[i] = d * data;
          simGetJointVelocity(h, &data);
          jointsVelocity[i] = d * data;
        }
        else
        {
#if MC_RTC_VERSION_MAJOR < 2
          auto qIdx = robot.jointIndexInMBC(i);
#else
          auto qIdx = robot.refJointIndexToQIndex(i);
#endif
          if(qIdx == -1)
          {
            continue;
          }
#if MC_RTC_VERSION_MAJOR < 2
          joints[i] = robot.mbc().q[qIdx][0];
#else
          joints[i] = robot.q()->value()(qIdx);
#endif
#if MC_RTC_VERSION_MAJOR < 2
          jointsVelocity[i] = robot.mbc().alpha[qIdx][0];
          jointsTorque[i] = robot.mbc().jointTorque[qIdx][0];
#else
          auto alphaIdx = robot.refJointIndexToQDotIndex(i);
          jointsVelocity[i] = robot.alpha()->value()(alphaIdx);
          jointsTorque[i] = robot.tau()->value()(alphaIdx);
#endif
        }
        gc.setEncoderValues(name, joints);
        gc.setEncoderVelocities(name, jointsVelocity);
        gc.setJointTorques(name, jointsTorque);
        for(auto & fs : fsHandles)
        {
          float force[3];
          float torque[3];
          auto h = fs.second;
          auto status = simReadForceSensor(h, force, torque);
          auto & out = wrenches[fs.first];
          out.force() << force[0], force[1], force[2];
          out.couple() << torque[0], torque[1], torque[2];
        }
        gc.setWrenches(name, wrenches);
      }
    }

    void updatePositionCommand(const mc_control::MCGlobalController & gc)
    {
      auto & robot = gc.robot(name);
      for(size_t i = 0; i < refJointToHandle.size(); ++i)
      {
        auto h = refJointToHandle[i].handle;
        if(h == -1)
        {
          continue;
        }
#if MC_RTC_VERSION_MAJOR < 2
        auto qIdx = robot.jointIndexInMBC(i);
#else
        auto qIdx = robot.refJointIndexToQIndex(i);
#endif
        if(qIdx == -1)
        {
          continue;
        }
        int _ = 0;
        auto jMode = simGetJointMode(h, &_);
        double d = refJointToHandle[i].reversed ? -1 : 1;
#if MC_RTC_VERSION_MAJOR < 2
        double q = d * robot.mbc().q[qIdx][0];
#else
        double q = d * robot.q()->value()(qIdx);
#endif
        if(jMode == sim_jointmode_force)
        {
          simSetJointTargetPosition(h, q);
        }
        else
        {
          simSetJointPosition(h, q);
        }
      }
    }

    void updateVelocityCommand(const mc_control::MCGlobalController & gc)
    {
      auto & robot = gc.robot(name);
      for(size_t i = 0; i < refJointToHandle.size(); ++i)
      {
        auto h = refJointToHandle[i].handle;
        if(h == -1)
        {
          continue;
        }
        double d = refJointToHandle[i].reversed ? -1 : 1;
#if MC_RTC_VERSION_MAJOR < 2
        auto alphaIdx = robot.jointIndexInMBC(i);
#else
        auto alphaIdx = robot.refJointIndexToQDotIndex(i);
#endif
        if(alphaIdx == -1)
        {
          continue;
        }
#if MC_RTC_VERSION_MAJOR < 2
        auto alpha = d * robot.mbc().alpha[alphaIdx][0];
#else
        auto alpha = d * robot.alpha()->value()(alphaIdx);
#endif
        simSetJointTargetVelocity(h, alpha);
      }
    }

    void updateTorqueCommand(const mc_control::MCGlobalController & gc)
    {
      auto & robot = gc.robot(name);
      for(size_t i = 0; i < refJointToHandle.size(); ++i)
      {
        auto h = refJointToHandle[i].handle;
        if(h == -1)
        {
          continue;
        }
#if MC_RTC_VERSION_MAJOR < 2
        auto alphaIdx = robot.jointIndexInMBC(i);
#else
        auto alphaIdx = robot.refJointIndexToQDotIndex(i);
#endif
        if(alphaIdx == -1)
        {
          continue;
        }
        double d = refJointToHandle[i].reversed ? -1 : 1;
#if MC_RTC_VERSION_MAJOR < 2
        auto tau = d * robot.mbc().jointTorque[alphaIdx][0];
#else
        auto tau = d * robot.tau()->value()(alphaIdx);
#endif
        if(tau > 0)
        {
          simSetJointForce(h, tau);
          simSetJointTargetVelocity(h, 1000);
        }
        else if(tau < 0)
        {
          simSetJointForce(h, -tau);
          simSetJointTargetVelocity(h, -1000);
        }
        else
        {
          simSetJointForce(h, 0);
          simSetJointTargetVelocity(h, 0);
        }
      }
    }

    /** Name of the robot in mc_rtc */
    std::string name;
    /** Handle for the model base */
    int rootHandle;
    /** Transformation offset between V-REP and mc_rtc */
    sva::PTransformd X_0_vrep;
    /** Handle for the joints */
    std::vector<VREPJoint> refJointToHandle;
    /** Handle for the force sensors */
    std::unordered_map<std::string, int> fsHandles;
    /** Joints' position */
    std::vector<double> joints;
    /** Joints' velocity */
    std::vector<double> jointsVelocity;
    /** Joints' torque */
    std::vector<double> jointsTorque;
    /** Force sensors reading */
    std::map<std::string, sva::ForceVecd> wrenches;
    /** Available body sensors */
    std::vector<std::string> bodySensors;
    /** Base position */
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    /** Base orientation */
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    /** Linear velocity reading for FloatingBase */
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    /** Angular velocity readings */
    std::map<std::string, Eigen::Vector3d> angularVelocities;
    /** Linear acceleration readings */
    std::map<std::string, Eigen::Vector3d> linearAccelerations;
  };
  /** VREP robots make the link between the VREP robot and an mc_rtc robot */
  std::vector<VREPRobot> robots_;
  bool first_ = true;
  size_t iter_ = 0;
  size_t frameskip_ = 1;
};

SimulationData::SimulationData(mc_control::MCGlobalController & controller) : gc_(controller), config_(gc_)
{
  auto makeVREPRobot = [this](const std::string & name, const std::string & suffix) {
    auto & robot = gc_.robots().robot(name);
#ifndef WIN32
    bfs::path tweaksFile =
        bfs::path(std::getenv("HOME")) / fmt::format(".config/mc_rtc/mc_vrep/{}.yaml", robot.module().name);
#else
    bfs::path tweaksFile =
        bfs::path(std::getenv("APPDATA")) / fmt::format("mc_rtc/mc_vrep/{}.yaml", robot.module().name);
#endif
    mc_rtc::Configuration tweaks;
    if(bfs::exists(tweaksFile))
    {
      mc_rtc::log::info("Using tweaks file for {} at {}", name, tweaksFile.string());
      tweaks.load(tweaksFile.string());
    }
    // First try to find the model base by using the robot's name
    auto rootHandle = utils::getModelBaseHandle(name);
    if(rootHandle == -1)
    {
      // Now we try to find it by checking the joint
      for(const auto & j : robot.mb().joints())
      {
        if(j.dof() == 1)
        {
          rootHandle = utils::getModelBaseHandle(j.name() + suffix);
          break;
        }
      }
    }
    if(rootHandle == -1)
    {
      mc_rtc::log::error(
          "mc_rtc is trying to control {} in CoppeliaSim but it cannot find it in the scene, either:\n1. The object "
          "does not have a model base in CoppeliaSim\n2. The object's name in mc_rtc does not match its name in "
          "CoppeliaSim\n3. The joints' names do not match between mc_rtc and CoppeliaSim",
          name);
      return;
    }
    std::vector<VREPRobot::VREPJoint> refJointToHandle(robot.refJointOrder().size());
    auto force_reversed = tweaks("force_reversed", std::vector<std::string>{});
    auto force_non_reversed = tweaks("force_non_reversed", std::vector<std::string>{});
    for(size_t i = 0; i < robot.refJointOrder().size(); ++i)
    {
      const auto & j = robot.refJointOrder()[i];
      auto h = simGetObjectHandle((j + suffix).c_str());
      if(h == -1)
      {
        mc_rtc::log::error("Joint {} in robot {} (mc_rtc) is not in the scene (looked for: {})", j, robot.name(),
                           j + suffix);
      }
      if(utils::getHandleRoot(h) != rootHandle)
      {
        mc_rtc::log::error("Joint {} in robot {} (mc_rtc) seems to be attached to a different robot in the scene "
                           "(matching model in CoppeliaSim: {}, searched for joint: {} which has parent {})",
                           j, robot.name(), simGetObjectName(rootHandle), j + suffix,
                           simGetObjectName(utils::getHandleRoot(h)));
        h = -1;
      }
      bool reversed = false;
      if(robot.hasJoint(j))
      {
        auto jIndex = robot.jointIndexByName(j);
        const auto & joint = robot.mb().joint(jIndex);
        if(joint.type() == rbd::Joint::Type::Rev || joint.type() == rbd::Joint::Type::Prism)
        {
          reversed = (joint.motionSubspace().array() < -1e-6).any();
        }
        if(std::find(force_reversed.begin(), force_reversed.end(), joint.name()) != force_reversed.end())
        {
          reversed = true;
        }
        if(std::find(force_non_reversed.begin(), force_non_reversed.end(), joint.name()) != force_non_reversed.end())
        {
          reversed = false;
        }
#if MC_RTC_VERSION_MAJOR < 2
        auto qIdx = robot.jointIndexInMBC(i);
#else
        auto qIdx = robot.refJointIndexToQIndex(i);
#endif
        if(h == -1 || qIdx == -1)
        {
          continue;
        }
        float encoder;
        simGetJointPosition(h, &encoder);
#if MC_RTC_VERSION_MAJOR < 2
        robot.mbc().q[qIdx][0] = encoder;
#else
        robot.q()->set(qIdx, encoder);
#endif
      }
      refJointToHandle[i] = {h, reversed};
    }
    sva::PTransformd X_0_vrep = sva::PTransformd::Identity();
    {
      /** Get a first value of the position */
      sva::PTransformd X_vrep_r = utils::getObjectPose(rootHandle);
      robot.posW(X_vrep_r);
      /** Now we find the first joint position, it must match between V-REP and mc_rtc even if the bodies origin have
       * moved */
      for(size_t i = 0; i < refJointToHandle.size(); ++i)
      {
        if(refJointToHandle[i].handle == -1)
        {
          continue;
        }
        const auto & jn = robot.refJointOrder()[i];
        if(!robot.hasJoint(jn))
        {
          continue;
        }
        auto jIndex = robot.jointIndexByName(jn);
        sva::PTransformd X_vrep_j = utils::getObjectPose(refJointToHandle[i].handle);
        auto parentIndex = robot.mb().predecessor(jIndex);
        sva::PTransformd X_0_j = robot.mb().transform(jIndex) * robot.mbc().bodyPosW[parentIndex];
        X_0_vrep = X_0_j.inv() * X_vrep_j;
        robot.posW(X_vrep_r * X_0_vrep);
        break;
      }
    }
    std::unordered_map<std::string, int> fsHandles;
    for(const auto & fs : robot.forceSensors())
    {
      const auto & name = fs.name();
      auto h = simGetObjectHandle((name + suffix).c_str());
      if(h == -1)
      {
        mc_rtc::log::error("Force sensor {} in robot {} (mc_rtc) is not in the scene (looked for {})", name,
                           robot.name(), name + suffix);
      }
      if(utils::getHandleRoot(h) != rootHandle)
      {
        mc_rtc::log::error(
            "Force sensor {} in robot {} (mc_rtc) seems to be attached to a different robot in the scene (matching "
            "model in CoppeliaSim: {}, searched for force sensor: {} which has parent {})",
            name, robot.name(), simGetObjectName(rootHandle), name + suffix, simGetObjectName(utils::getHandleRoot(h)));
        h = -1;
      }
      fsHandles[name] = h;
    }
    std::vector<std::string> bodySensors;
    for(const auto & bs : robot.bodySensors())
    {
      const auto & name = bs.name();
      if(name == "FloatingBase")
      {
        continue;
      }
      auto h = simGetObjectHandle((name + suffix).c_str());
      if(h == -1)
      {
        mc_rtc::log::error("Body sensor {} in robot {} is not in the scene (looked for {})", name, robot.name(),
                           name + suffix);
        continue;
      }
      auto gyro = name + "_GyroSensor" + suffix;
      auto h_gyro = simGetObjectHandle(gyro.c_str());
      if(h_gyro == -1)
      {
        mc_rtc::log::error("Body sensor {} in robot {}, gyrometer is not in the scene (looked for {})", name,
                           robot.name(), gyro);
        continue;
      }
      if(utils::getHandleRoot(h) != rootHandle || utils::getHandleRoot(h_gyro) != rootHandle)
      {
        mc_rtc::log::error(
            "Body sensor {} in robot {} (mc_rtc) seems to be attached to a different robot in the scene (matching "
            "model in CoppeliaSim: {}, searched for force sensor: {} which has parent {})",
            name, robot.name(), simGetObjectName(rootHandle), name + suffix, simGetObjectName(utils::getHandleRoot(h)));
        continue;
      }
      bodySensors.push_back(name);
    }
    robots_.emplace_back(robot.name(), rootHandle, X_0_vrep, refJointToHandle, fsHandles, bodySensors);
  };
  makeVREPRobot(gc_.controller().robot().name(), "");
  if(robots_.empty())
  {
    mc_rtc::log::critical("Main robot is not in the scene, stopping the simulation now");
    simulation_stopped = true;
  }
  for(const auto & r : config_.extras)
  {
    if(r.name.size())
    {
      makeVREPRobot(r.name, r.suffix);
    }
  }
}

void SimulationData::run()
{
  auto & ctl = gc_.controller();
  if(first_)
  {
    config_.simulationTimestep = simGetSimulationTimeStep();
#if MC_RTC_VERSION_MAJOR < 2
    frameskip_ = std::round(ctl.timeStep / config_.simulationTimestep);
#else
    frameskip_ = std::round(ctl.solver().dt() / config_.simulationTimestep);
#endif
    mc_rtc::log::info("[mc_vrep] Frameskip: {}", frameskip_);
  }
  if(iter_++ % frameskip_ != 0 || iter_ < 5)
  {
    return;
  }
  /** Update sensors */
  for(auto & r : robots_)
  {
    r.updateSensors(gc_);
  }
  if(first_)
  {
    for(const auto & r : robots_)
    {
      if(r.name == ctl.robot().name())
      {
        continue;
      }
      auto & robot = ctl.robot(r.name);
      const auto & rjo = robot.refJointOrder();
      for(size_t i = 0; i < rjo.size(); ++i)
      {
        const auto & j = rjo[i];
#if MC_RTC_VERSION_MAJOR < 2
        auto qIdx = robot.jointIndexInMBC(i);
#else
        auto qIdx = robot.refJointIndexToQIndex(i);
#endif
        if(r.refJointToHandle[i].handle == -1 || qIdx == -1)
        {
          continue;
        }
#if MC_RTC_VERSION_MAJOR < 2
        robot.mbc().q[qIdx][0] = r.joints[i];
#else
        robot.q()->set(qIdx, r.joints[i]);
#endif
      }
      robot.posW({r.orientation, r.position});
    }
    gc_.init(ctl.robot().encoderValues(), sva::PTransformd{robots_[0].orientation, robots_[0].position});
    gc_.running = true;
    first_ = false;
  }
  /** Run the controller */
  if(gc_.run())
  {
    /** Update the control */
    for(auto & r : robots_)
    {
      if(config_.velocityControl)
      {
        r.updateVelocityCommand(gc_);
      }
      else if(config_.torqueControl)
      {
        r.updateTorqueCommand(gc_);
      }
      else
      {
        r.updatePositionCommand(gc_);
      }
    }
  }
}

static std::unique_ptr<SimulationData> simulation_data;

static mc_rtc::gui::StateBuilder & get_gui()
{
  if(gc_ptr)
  {
#if MC_RTC_VERSION_MAJOR < 2
    auto gui_ptr = gc_ptr->controller().gui();
    if(!gui_ptr)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("GUI must be enabled for McRtc plugin to work");
    }
    return *gui_ptr;
#else
    return gc_ptr->controller().gui();
#endif
  }
  if(!gc_config_ptr)
  {
    gc_config_ptr.reset(new mc_control::MCGlobalController::GlobalConfiguration(""));
  }
  if(!default_gui_ptr)
  {
    const auto & c = *gc_config_ptr;
    server_ptr.reset(
        new mc_control::ControllerServer(c.timestep, c.gui_timestep, c.gui_server_pub_uris, c.gui_server_rep_uris));
    default_gui_ptr.reset(new mc_rtc::gui::StateBuilder());
  }
  return *default_gui_ptr;
}

static void reset_gui()
{
  auto & gui = get_gui();
  simulation_paused = false;
  simulation_steps = 0;
  simulation_stopped = false;
  simulation_reset = false;
  gui.addElement({"VREP"}, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Label("Simulation control", []() { return ""; }),
                 mc_rtc::gui::Button("Start",
                                     []() {
                                       simulation_started = true;
                                       simulation_paused = false;
                                     }),
                 mc_rtc::gui::Button("Pause",
                                     []() {
                                       if(simulation_started)
                                       {
                                         simulation_paused = !simulation_paused;
                                       }
                                     }),
                 mc_rtc::gui::Button("Stop",
                                     []() {
                                       if(simulation_started)
                                       {
                                         simulation_stopped = true;
                                       }
                                     }),
                 mc_rtc::gui::Button("Reset", []() {
                   if(simulation_started)
                   {
                     simulation_reset = true;
                   }
                 }));
  gui.addElement({"VREP"}, mc_rtc::gui::Checkbox(
                               "Step by step", []() { return simulation_step_by_step; },
                               []() {
                                 simulation_step_by_step = !simulation_step_by_step;
                                 simulation_steps = 0;
                               }));
  double dt = gc_config_ptr->timestep;
  auto label = [&](size_t i) { return fmt::format("+{}ms", i * std::ceil(dt * 1000)); };
  gui.addElement({"VREP"}, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Button(label(1), []() { simulation_steps = 1; }),
                 mc_rtc::gui::Button(label(5), []() { simulation_steps = 5; }),
                 mc_rtc::gui::Button(label(10), []() { simulation_steps = 10; }),
                 mc_rtc::gui::Button(label(25), []() { simulation_steps = 25; }),
                 mc_rtc::gui::Button(label(50), []() { simulation_steps = 50; }),
                 mc_rtc::gui::Button(label(100), []() { simulation_steps = 100; }));
}

static void reset_gc()
{
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error("mc_vrep was compiled with {} but mc_rtc is at version {}, you might "
                       "face subtle issues or unexpected crashes, please recompile mc_vrep",
                       mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }
  if(server_ptr)
  {
    server_ptr.reset(nullptr);
    default_gui_ptr.reset(nullptr);
  }
  gc_config_ptr.reset(new mc_control::MCGlobalController::GlobalConfiguration(""));
  gc_ptr.reset(new mc_control::MCGlobalController(*gc_config_ptr));
  reset_gui();
}

static mc_control::MCGlobalController & gc()
{
  if(!gc_ptr)
  {
    reset_gc();
  }
  return *gc_ptr;
}

} // namespace mc_vrep

SIM_DLLEXPORT unsigned char simStart(void *, int)
{
  // Dynamically load and bind CoppelisSim functions:
  // 1. Figure out this plugin's directory:
  char curDirAndFile[1024];
#ifdef _WIN32
#  ifdef QT_COMPIL
  _getcwd(curDirAndFile, sizeof(curDirAndFile));
#  else
  GetModuleFileName(NULL, curDirAndFile, 1023);
  PathRemoveFileSpec(curDirAndFile);
#  endif
#else
  getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

  std::string currentDirAndPath(curDirAndFile);
  // 2. Append the CoppelisSim library's name:
  std::string temp(currentDirAndPath);
#ifdef _WIN32
  temp += "\\coppeliaSim.dll";
#elif defined(__linux)
  temp += "/libcoppeliaSim.so";
#elif defined(__APPLE__)
  temp += "/libcoppeliaSim.dylib";
#endif /* __linux || __APPLE__ */
  // 3. Load the CoppelisSim library:
  simLib = loadSimLibrary(temp.c_str());
  if(simLib == NULL)
  {
    printf("simExtMcRtc: error: could not find or correctly load the CoppeliaSim library. Cannot start the "
           "plugin.\n"); // cannot use simAddLog here.
    return (0); // Means error, CoppelisSim will unload this plugin
  }
  if(getSimProcAddresses(simLib) == 0)
  {
    printf("simMcRtc: error: could not find all required functions in the CoppeliaSim library. Cannot "
           "start the plugin.\n"); // cannot use simAddLog here.
    unloadSimLibrary(simLib);
    return (0); // Means error, CoppelisSim will unload this plugin
  }
  simSetModuleInfo("McRtc", sim_moduleinfo_verbosity, nullptr, sim_verbosity_traceall);
  simSetModuleInfo("McRtc", sim_moduleinfo_statusbarverbosity, nullptr, sim_verbosity_traceall);
  mc_vrep::sink = std::make_shared<mc_vrep::LogSink>();
  mc_vrep::success_sink = std::make_shared<mc_vrep::SuccessSink>(mc_vrep::sink);
  mc_rtc::log::details::info().sinks().push_back(mc_vrep::sink);
  mc_rtc::log::details::success().sinks().push_back(mc_vrep::success_sink);
  mc_rtc::log::details::cerr().sinks().push_back(mc_vrep::sink);
  mc_vrep::reset_gui();

  return PLUGIN_VERSION;
}

SIM_DLLEXPORT void simEnd() {}

SIM_DLLEXPORT void * simMessage(int message, int * auxiliaryData, void * customData, int * replyData)
{
  static bool refreshDlgFlag = true;
  int errorModeSaved;
  simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
  simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
  void * retVal = NULL;

  if(message == sim_message_eventcallback_refreshdialogs)
    refreshDlgFlag = true; // CoppelisSim dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

  if(message == sim_message_eventcallback_instancepass)
  {
    // This is called every time CoppeliaSim runs its main loop
    for(const auto & msg : mc_vrep::sink->msgs())
    {
      auto levelToVerbosity = [](mc_vrep::LogSink::Level level) {
        using Level = mc_vrep::LogSink::Level;
        switch(level)
        {
          case Level::SUCCESS:
          case Level::INFO:
            return sim_verbosity_infos;
          case Level::WARNING:
            return sim_verbosity_warnings;
          case Level::ERROR:
          case Level::CRITICAL:
            return sim_verbosity_errors;
          default:
            return sim_verbosity_none;
        }
      };
      simAddLog("McRtc", levelToVerbosity(msg.level), msg.msg.c_str());
    }
    mc_vrep::sink->clear();

    auto simState = simGetSimulationState();
    if(simState == sim_simulation_stopped && mc_vrep::simulation_started)
    {
      simStartSimulation();
    }
    if(simState == sim_simulation_paused && !mc_vrep::simulation_paused)
    {
      simStartSimulation();
    }
    if(simState > sim_simulation_paused && mc_vrep::simulation_paused)
    {
      simPauseSimulation();
    }
    if(simState >= sim_simulation_paused && (mc_vrep::simulation_stopped || mc_vrep::simulation_reset))
    {
      simStopSimulation();
    }
    if(mc_vrep::simulation_paused)
    {
      mc_vrep::gc().running = false;
      mc_vrep::gc().run();
      mc_vrep::gc().running = true;
    }
    if(mc_vrep::server_ptr && mc_vrep::default_gui_ptr)
    {
      mc_vrep::server_ptr->publish(*mc_vrep::default_gui_ptr);
      mc_vrep::server_ptr->handle_requests(*mc_vrep::default_gui_ptr);
    }
  }

  if(message == sim_message_eventcallback_mainscriptabouttobecalled)
  {
    // This is called before CoppeliaSim runs the simulation main loop, by writing something other than -1 in replyData
    // we can prevent the main script execution
    if(mc_vrep::simulation_started && mc_vrep::simulation_step_by_step)
    {
      if(mc_vrep::simulation_steps == 0)
      {
        mc_vrep::gc().running = false;
        mc_vrep::gc().run();
        mc_vrep::gc().running = true;
        replyData[0] = 1;
      }
      else
      {
        mc_vrep::simulation_steps--;
      }
    }
  }

  if(message == sim_message_eventcallback_simulationabouttostart)
  { // Simulation is about to start
    if(mc_vrep::simulation_started)
    {
      mc_vrep::simulation_data.reset(new mc_vrep::SimulationData(mc_vrep::gc()));
    }
  }

  if(message == sim_message_eventcallback_modulehandle)
  { // A script called simHandleModule (by default the main script). Is only called during simulation.
    if((customData == NULL)
       || (_stricmp("McRtc", (char *)customData) == 0)) // is the command also meant for this plugin?
    {
      // we arrive here only while a simulation is running
      if(mc_vrep::simulation_started)
      {
        mc_vrep::simulation_data->run();
      }
    }
  }

  if(message == sim_message_eventcallback_simulationended)
  { // Simulation just ended
    if(mc_vrep::simulation_started)
    {
      mc_vrep::simulation_data.reset();
      if(mc_vrep::simulation_reset)
      {
        mc_vrep::reset_gc();
      }
      else
      {
        mc_vrep::simulation_started = false;
        mc_vrep::gc_ptr.reset(nullptr);
        mc_vrep::reset_gui();
      }
    }
  }

  if((message == sim_message_eventcallback_guipass) && refreshDlgFlag)
  { // handle refresh of the plugin's dialogs
    // ...
    refreshDlgFlag = false;
  }

  simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
  return (retVal);
}
