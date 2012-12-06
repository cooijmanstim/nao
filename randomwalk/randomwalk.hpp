#pragma once

#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>

#include <boost/math/constants/constants.hpp>

const float PI = boost::math::constants::pi<float>();

namespace AL { class ALBroker; }

class RandomWalk : public AL::ALModule
{
public:
  RandomWalk(boost::shared_ptr<AL::ALBroker> broker, const std::string &name);
  ~RandomWalk();
  void init();

  void start();

  boost::uniform_real<> distribution;
  boost::mt19937 generator;
  boost::variate_generator<boost::mt19937&, boost::uniform_real<> > random;

  AL::ALMotionProxy motion;
  AL::ALMemoryProxy memory;
  AL::ALRobotPostureProxy posture;
};
