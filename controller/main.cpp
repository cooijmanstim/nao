#include <iostream>

#include <alcommon/alproxy.h>
#include <alvalue/alvalue.h>

using namespace std;

int main(int argc, char**argv)
{
  assert(argc >= 2);
  const string host = argv[1];
  const int port = 9559;
  const string methodName = "start";

  AL::ALProxy observer("Observer", host, port);
  observer.callVoid(methodName);

  AL::ALProxy randomwalk("RandomWalk", host, port);
  randomwalk.callVoid(methodName);

  return 0;
}
