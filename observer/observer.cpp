#include "observer.hpp"

#include <iostream>
#include <time.h>
#include <unistd.h>
#include <math.h>

#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <alcommon/almodule.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>

#include <qi/log.hpp>

const std::string Observer::sensor_names[Observer::sensor_count] = {
  "Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/Position/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/Position/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/Position/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/Position/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/Position/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/Position/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/Position/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/GyrRef/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccY/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/LShoulderRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/LElbowYaw/Hardness/Actuator/Value",
  "Device/SubDeviceList/LElbowRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/RShoulderPitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/RShoulderRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/RElbowYaw/Hardness/Actuator/Value",
  "Device/SubDeviceList/RElbowRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/LHipRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/LHipPitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/LKneePitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/LAnklePitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/LAnkleRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/RHipRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/RHipPitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/RKneePitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/RAnklePitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/RAnkleRoll/Hardness/Actuator/Value",
};

const std::string Observer::actuator_names[Observer::actuator_count] = {
  "Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value",
  "Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value",
  "Device/SubDeviceList/LElbowYaw/Position/Actuator/Value",
  "Device/SubDeviceList/LElbowRoll/Position/Actuator/Value",
  "Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value",
  "Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value",
  "Device/SubDeviceList/RElbowYaw/Position/Actuator/Value",
  "Device/SubDeviceList/RElbowRoll/Position/Actuator/Value",
  "Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value",
  "Device/SubDeviceList/LHipRoll/Position/Actuator/Value",
  "Device/SubDeviceList/LHipPitch/Position/Actuator/Value",
  "Device/SubDeviceList/LKneePitch/Position/Actuator/Value",
  "Device/SubDeviceList/LAnklePitch/Position/Actuator/Value",
  "Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value",
  "Device/SubDeviceList/RHipRoll/Position/Actuator/Value",
  "Device/SubDeviceList/RHipPitch/Position/Actuator/Value",
  "Device/SubDeviceList/RKneePitch/Position/Actuator/Value",
  "Device/SubDeviceList/RAnklePitch/Position/Actuator/Value",
  "Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value",
  "Device/SubDeviceList/LShoulderPitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/LShoulderRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/LElbowYaw/Hardness/Actuator/Value",
  "Device/SubDeviceList/LElbowRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/RShoulderPitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/RShoulderRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/RElbowYaw/Hardness/Actuator/Value",
  "Device/SubDeviceList/RElbowRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/LHipRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/LHipPitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/LKneePitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/LAnklePitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/LAnkleRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/RHipRoll/Hardness/Actuator/Value",
  "Device/SubDeviceList/RHipPitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/RKneePitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/RAnklePitch/Hardness/Actuator/Value",
  "Device/SubDeviceList/RAnkleRoll/Hardness/Actuator/Value",
};

const string Observer::comment_prefix = "# ";
const string Observer::column_separator = " ";
const string Observer::record_separator = "\n";

Observer::Observer(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) : AL::ALModule(broker, name) {
  current_values_initialized = false;
  current_time = 0;

  setModuleDescription("observe in and outputs");

  functionName("start", getName(), "start observing");
  BIND_METHOD(Observer::start);

  functionName("stop", getName(), "stop observing");
  BIND_METHOD(Observer::start);
}

Observer::~Observer() {
}

void Observer::preprocess() {
  {
    boost::unique_lock<boost::mutex> lock(condition_mutex);

    for (int i = 0; i < actuator_count; i++)
      current_actuator_values[i] = *actuator_pointers[i];
  }

  if (current_values_initialized)
    condition.notify_all();
}

void Observer::postprocess() {
  {
    boost::unique_lock<boost::mutex> lock(condition_mutex);
    
    // FIXME: this contains the assumption that the simulation runs at 1x
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    current_time = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

    has_goal = motion.moveIsActive() ? 1 : 0;
    if (has_goal == 1) {
      // need to translate and rotate the absolute goal position to be relative to the robot's
      // position and with the x dimension pointing in the direction the robot is facing
      vector<float> worldpose = motion.getRobotPosition(false);
      float worldgoal[] = { *worldgoal_pointers[0], *worldgoal_pointers[1] };
      float robotgoalx = worldgoal[0] - worldpose[0], robotgoaly = worldgoal[1] - worldpose[1];
      float rotcos = cos(-worldpose[2]), rotsin = sin(-worldpose[2]);
      goalx = robotgoalx * rotcos - robotgoaly * rotsin;
      goaly = robotgoalx * rotsin + robotgoaly * rotcos;
    } else {
      // if no goal, then no reward, but these samples are still useful for unsupervised learning
      goalx = goaly = NaN;
    }

    for (int i = 0; i < sensor_count; i++)
      current_sensor_values[i] = *sensor_pointers[i];
  }

  current_values_initialized = true;
}

void Observer::initialize_pointers(int n, const string keys[], float* pointers[]) {
  for (int i = 0; i < n; i++)
    pointers[i] = (float*)memory.getDataPtr(keys[i]);
}

void Observer::record_header(ostream& stream) {
  boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
  stream << comment_prefix << "random walk starting " << now << " with " << sensor_count << " sensors, " << actuator_count << " actuators" << endl;
  stream << "timestamp" << column_separator;
  stream << "goalx" << column_separator << "goaly" << column_separator;
  for (int i = 0; i < sensor_count; i++)
    stream << sensor_names[i] << column_separator;
  for (int i = 0; i < actuator_count; i++)
    stream << actuator_names[i] << column_separator;
  stream << record_separator;
}

void Observer::record_data(string filepath) {
  boost::iostreams::stream_buffer<boost::iostreams::file_sink> buffer(filepath);
  ostream stream(&buffer);
  record_header(stream);

  time_t previous_time = -1;

  while (true) {
    boost::unique_lock<boost::mutex> lock(condition_mutex);

    while (current_time <= previous_time && !stopping)
      condition.wait(lock);
    if (stopping)
      return;

    previous_time = current_time;

    stream << current_time << column_separator;
    stream << has_goal << column_separator;
    stream << goalx << column_separator << goaly << column_separator;
    for (int i = 0; i < sensor_count; i++)
      stream << current_sensor_values[i] << column_separator;
    for (int i = 0; i < actuator_count; i++)
      stream << current_actuator_values[i] << column_separator;
    stream << record_separator;
  }
}

void Observer::init() {
  initialize_pointers(sensor_count, sensor_names, sensor_pointers);
  initialize_pointers(actuator_count, actuator_names, actuator_pointers);
  worldgoal_pointers[0] = (float*)memory.getDataPtr("Module/RandomWalk/goal/x");
  worldgoal_pointers[1] = (float*)memory.getDataPtr("Module/RandomWalk/goal/y");
}

void Observer::start() {
  string filepath = "/tmp/robotwalk_" + boost::lexical_cast<string>(getpid());

  stopping = false;
  boost::thread data_storage_thread(boost::bind(&Observer::record_data, this, _1), filepath);

  try {
    boost::shared_ptr<AL::ALModule> m = getParentBroker()->getProxy("ALMotion")->getModule();
    m->atPreProcess(boost::bind(&Observer::preprocess, this));
    m->atPostProcess(boost::bind(&Observer::postprocess, this));
  } catch(const AL::ALError& e) {
    cout << "error registering pre/post process handlers" << endl;
    throw e;
  }
}

void Observer::stop() {
  stopping = true;
  condition.notify_all();
}
