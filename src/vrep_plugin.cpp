#include "simLib.h"

static LIBRARY simLib; // the CoppelisSim library that we will dynamically load and bind

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

// For the configuration
#include "vrep_simulation_configuration.h"

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
/** True when we try to reset the GC from the GUI */
static bool want_reset_gc = false;

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
              const std::vector<VREPJoint> & refJointToHandle,
              const std::unordered_map<std::string, int> & fsHandles)
    : name(name), rootHandle(rootHandle), refJointToHandle(refJointToHandle), fsHandles(fsHandles)
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
    }

    void updateSensors(mc_control::MCGlobalController & gc)
    {
      const auto & robot = gc.robot(name);
      const auto & rjo = robot.refJointOrder();
      {
        float pos[7];
        // pos is tx,ty,tz,qx,qy,qz,qw
        simGetObjectPose(rootHandle, -1, pos);
        position << pos[0], pos[1], pos[2];
        orientation.w() = pos[6];
        orientation.x() = pos[3];
        orientation.y() = pos[4];
        orientation.z() = pos[5];
        orientation = orientation.inverse();
        gc.setSensorPosition(name, position);
        gc.setSensorOrientation(name, orientation);
      }
      {
        float linV[3];
        float accV[3];
        simGetObjectVelocity(rootHandle, linV, accV);
        velocity.linear() << linV[0], linV[1], linV[2];
        velocity.angular() << accV[0], accV[1], accV[2];
        gc.setSensorLinearVelocity(name, velocity.linear());
        gc.setSensorAngularVelocity(name, velocity.angular());
      }
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
    /** Base position */
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    /** Base orientation */
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    /** Base velocity */
    sva::MotionVecd velocity = sva::MotionVecd::Zero();
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
    const auto & robot = gc_.robots().robot(name);
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
          "CoppeliaSim\n3. The joints' names do not match between mc_rtc and CoppeliaSim");
      return;
    }
    std::vector<VREPRobot::VREPJoint> refJointToHandle(robot.refJointOrder().size());
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
      }
      refJointToHandle[i] = {h, reversed};
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
    robots_.emplace_back(robot.name(), rootHandle, refJointToHandle, fsHandles);
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

static void reset_gc()
{
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error("mc_vrep was compiled with {} but mc_rtc is at version {}, you might "
                       "face subtle issues or unexpected crashes, please recompile mc_vrep",
                       mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }
  gc_ptr.reset(new mc_control::MCGlobalController());
  simulation_started = simulation_reset;
  simulation_paused = false;
  simulation_steps = 0;
  simulation_stopped = false;
  simulation_reset = false;
  want_reset_gc = false;
#if MC_RTC_VERSION_MAJOR < 2
  auto gui_ptr = gc_ptr->controller().gui();
  if(!gui_ptr)
  {
    return;
  }
  auto & gui = *gui_ptr;
#else
  auto & gui = gc_ptr->controller().gui();
#endif
  gui.addElement({"VREP"}, mc_rtc::gui::Button("Reload mc_rtc", []() { want_reset_gc = true; }));
  gui.addElement({"VREP"}, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Label("Simulation control", []() { return ""; }),
                 mc_rtc::gui::Button("Start", []() { simulation_started = true; }),
                 mc_rtc::gui::Button("Pause", []() { simulation_paused = !simulation_paused; }),
                 mc_rtc::gui::Button("Stop", []() { simulation_stopped = true; }),
                 mc_rtc::gui::Button("Reset", []() { simulation_reset = true; }));
  gui.addElement({"VREP"}, mc_rtc::gui::Checkbox("Step by step", []() { return simulation_step_by_step; },
                                                 []() {
                                                   simulation_step_by_step = !simulation_step_by_step;
                                                   simulation_steps = 0;
                                                 }));
  double dt = gc_ptr->controller().solver().dt();
  auto label = [&](size_t i) { return fmt::format("+{}ms", i * std::ceil(dt * 1000)); };
  gui.addElement({"VREP"}, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Button(label(1), []() { simulation_steps = 1; }),
                 mc_rtc::gui::Button(label(5), []() { simulation_steps = 5; }),
                 mc_rtc::gui::Button(label(10), []() { simulation_steps = 10; }),
                 mc_rtc::gui::Button(label(25), []() { simulation_steps = 25; }),
                 mc_rtc::gui::Button(label(50), []() { simulation_steps = 50; }),
                 mc_rtc::gui::Button(label(100), []() { simulation_steps = 100; }));
}

static mc_control::MCGlobalController & gc()
{
  if(!gc_ptr)
  {
    reset_gc();
  }
  return *gc_ptr;
}

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

  reset_gc();

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
    auto simState = simGetSimulationState();
    if(want_reset_gc)
    {
      if(simState != sim_simulation_stopped)
      {
        mc_rtc::log::critical("mc_rtc cannot be reset while the simulation is running");
      }
      else
      {
        reset_gc();
      }
    }
    if(simState == sim_simulation_stopped && simulation_started)
    {
      simStartSimulation();
    }
    if(simState == sim_simulation_paused && !simulation_paused)
    {
      simStartSimulation();
    }
    if(simState > sim_simulation_paused && simulation_paused)
    {
      simPauseSimulation();
    }
    if(simState >= sim_simulation_paused && (simulation_stopped || simulation_reset))
    {
      simStopSimulation();
    }
    if(!simulation_started || simulation_paused)
    {
      gc().running = false;
      gc().run();
      gc().running = true;
    }
  }

  if(message == sim_message_eventcallback_mainscriptabouttobecalled)
  {
    // This is called before CoppeliaSim runs the simulation main loop, by writing something other than -1 in replyData
    // we can prevent the main script execution
    if(simulation_step_by_step)
    {
      if(simulation_steps == 0)
      {
        gc().running = false;
        gc().run();
        gc().running = true;
        replyData[0] = 1;
      }
      else
      {
        simulation_steps--;
      }
    }
  }

  if(message == sim_message_eventcallback_simulationabouttostart)
  { // Simulation is about to start
    simulation_data.reset(new SimulationData(gc()));
  }

  if(message == sim_message_eventcallback_modulehandle)
  { // A script called simHandleModule (by default the main script). Is only called during simulation.
    if((customData == NULL)
       || (_stricmp("McRtc", (char *)customData) == 0)) // is the command also meant for this plugin?
    {
      // we arrive here only while a simulation is running
      simulation_data->run();
    }
  }

  if(message == sim_message_eventcallback_simulationended)
  { // Simulation just ended
    simulation_data.reset();
    reset_gc();
  }

  if((message == sim_message_eventcallback_guipass) && refreshDlgFlag)
  { // handle refresh of the plugin's dialogs
    // ...
    refreshDlgFlag = false;
  }

  simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
  return (retVal);
}
