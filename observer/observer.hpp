#pragma once

#include <alcommon/almodule.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <limits>

using namespace std;

const float NaN = numeric_limits<float>::quiet_NaN();

namespace AL { class ALBroker; }

class Observer : public AL::ALModule
{
public:
  Observer(boost::shared_ptr<AL::ALBroker> broker, const std::string &name);
  ~Observer();
  void init();

static const string comment_prefix;
static const string column_separator;
static const string record_separator;

static const int sensor_count = 52;
static const int actuator_count = 38;

static const std::string sensor_names[];
static const std::string actuator_names[];

float *sensor_pointers[sensor_count];
float *actuator_pointers[actuator_count];
float *worldgoal_pointers[2];

// these are filled in in two steps and then written out as a state-action pair.
float current_sensor_values[sensor_count];
float current_actuator_values[actuator_count];
bool current_values_initialized;

time_t current_time;
int has_goal;
float goalx, goaly;

boost::mutex condition_mutex;
boost::condition_variable condition;

AL::ALMotionProxy motion;
AL::ALMemoryProxy memory;

void preprocess();
void postprocess();
void initialize_pointers(int n, const string keys[], float* pointers[]);
void record_header(ostream& stream);
void record_data(string filepath);

bool stopping;
void start();
void stop();

};

