#include <ros/ros.h>
#include <agv_exploration_test/AgvExplorerTest.h>

using namespace ros;
using namespace actionlib;
//using namespace agv_exploration;

int main(int argc, char **argv)
{
  init(argc, argv, "agv_exploration_test");
  NodeHandle n;

  AgvExplorerTest agvExplorerTest(n);

  return 0;
}