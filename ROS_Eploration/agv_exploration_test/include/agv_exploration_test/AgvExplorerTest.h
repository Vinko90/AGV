#ifndef AGVEXPLORERTEST_H_
#define AGVEXPLORERTEST_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <string.h>
#include <vector>

using namespace std;

#define TEST_TIMEOUT 1800
#define WORKING_FOLDER_DEFAULT "/home/agv/gitlab/catkin_ws/src/"
#define OUTPUT_FOLDER_DEFAULT "/home/agv/gitlab/"

class AgvExplorerTest {
    
    public:
        /**
         * Constructor.
         * @param n The ROS node handle.
         */
        AgvExplorerTest(ros::NodeHandle n);

        /**
         * Destructor.
         */
        ~AgvExplorerTest();

    private:
        /**
         * The name of the folder in which the test results from the current test will be output. The folder name is
         * "test_" + the current timestamp.
         */
        string _outputTestFolderName;

        /**
         * The amount of time for which the test node will wait for each map. If the exploration goal for a map is not
         * completed or aborted by that time, then the goal will be canceled and the test will continue. The value can
         * be set by setting the "test_timeout" ROS param.
         */
        int _testTimeout;

        /**
         * The location of the working folder containing the agv-exploration repository. The value can be set by setting
         * the "working_folder" ROS param. By default the value is set to "/home/ivamil/gitlab/catkin_ws/src/".
         */
        string _workingFolder;

        /**
         * The location of the folder in which the test results will be output. The value can be set by setting the
         * "output_folder" ROS param. By default the value is set to "/home/ivamil/".
         */
        string _outputFolder;

        /**
         * The list of launch files that contain test scenarios.
         */
        vector<string> _launchFileNames;

        /**
         * Retrieves the relevant launch files and sets them in _launchFileNames property.
         */
        void retriveLaunchFiles();

        /**
         * Starts the test simulation with all the retrieved launch files.
         */
        void startTestSimulation();

        /**
         * Gets the current timestamp.
         * @return The current timestamp.
         */
        string getCurrentTimeStamp();

        /**
         * Kills all running nodes except for the test node.
         */
        void killAllNodes();

        /**
         * Gets the formatted timer time.
         * @param value The value of the timer to be formatted.
         * @return The formatted value.
         */
        string getFormattedTimerTime(int value);
};

#endif // AGVEXPLORERTEST_H_