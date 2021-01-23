#include <ros/ros.h>
#include <agv_exploration/AgvExplorer.h>

using namespace ros;
using namespace actionlib;
using namespace agv_exploration;

int main(int argc, char **argv)
{
  init(argc, argv, "agv_exploration");
  NodeHandle n;

  AgvExplorer agvExplorer(n);

  return 0;
}