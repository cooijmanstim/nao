#include "randomwalk.hpp"
#include <qi/log.hpp>
#include <math.h>

using namespace std;

RandomWalk::RandomWalk(boost::shared_ptr<AL::ALBroker> broker,
                       const string& name)
  : AL::ALModule(broker, name),
    distribution(-PI, PI),
    generator(time(0)),
    random(generator, distribution)
{
  setModuleDescription("walk randomly forever");

  functionName("start", getName(), "Just do it.");
  BIND_METHOD(RandomWalk::start);
}

RandomWalk::~RandomWalk()
{
}

void RandomWalk::init()
{
}

void RandomWalk::start() {
  // TODO: less stiff? maybe random stiffness?
  motion.setStiffnesses("Body", 1.0f);
  sleep(1);

  while (true) {
    if (!posture.goToPosture("Stand", 1))
      throw "disappoint";

    float x = random();
    float y = random();
    float theta = random();

    vector<float> pose_before = motion.getRobotPosition(false);
    qiLogInfo("RandomWalk", "walking from (%f %f %f) to (%f %f %f)", pose_before[0], pose_before[1], pose_before[2], x, y, theta);

    // express the goal in FRAME_WORLD and store it in memory so that Observer can access it
    float rotcos = cos(pose_before[2]), rotsin = sin(pose_before[2]);
    float worldx = x * rotcos - y * rotsin + pose_before[0];
    float worldy = x * rotsin + y * rotcos + pose_before[1];
    memory.insertData("Module/RandomWalk/goal/x", worldx);
    memory.insertData("Module/RandomWalk/goal/y", worldy);

    motion.moveInit();
    motion.moveTo(x, y, theta);
    motion.waitUntilMoveIsFinished();

    sleep(1);
  }
}
