#include "simLib.h"

static LIBRARY simLib; // the CoppelisSim library that we will dynamically load and bind

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>

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

static void reset_gc()
{
  gc_ptr = std::make_unique<mc_control::MCGlobalController>();
  simulation_started = simulation_reset;
  simulation_paused = false;
  simulation_steps = 0;
  simulation_stopped = false;
  simulation_reset = false;
  want_reset_gc = false;
  auto & gui = gc_ptr->controller().gui();
  gui.addElement({"VREP"}, mc_rtc::gui::Button("Reload mc_rtc", []() { want_reset_gc = true; }));
  gui.addElement({"VREP"}, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Label("Simulation control", []() { return ""; }),
                 mc_rtc::gui::Button("Start", []() { simulation_started = true; }),
                 mc_rtc::gui::Button("Pause", []() { simulation_paused = !simulation_paused; }),
                 mc_rtc::gui::Button("Stop", []() { simulation_stopped = true; }),
                 mc_rtc::gui::Button("Reset", []() { simulation_reset = true; }));
  gui.addElement({"VREP"}, mc_rtc::gui::Checkbox(
                               "Step by step", []() { return simulation_step_by_step; },
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

  if(message == sim_message_eventcallback_simulationabouttostart)
  { // Simulation is about to start
  }

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

  if(message == sim_message_eventcallback_modulehandle)
  { // A script called simHandleModule (by default the main script). Is only called during simulation.
    if((customData == NULL)
       || (_stricmp("McRtc", (char *)customData) == 0)) // is the command also meant for this plugin?
    {
      // we arrive here only while a simulation is running
      gc().run();
    }
  }

  if(message == sim_message_eventcallback_simulationended)
  { // Simulation just ended
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
