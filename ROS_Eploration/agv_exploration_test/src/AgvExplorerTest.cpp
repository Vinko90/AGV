#include <agv_exploration_test/AgvExplorerTest.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <agv_exploration/StartExplorationGoal.h>
#include <agv_exploration/StartExplorationAction.h>
#include "boost/filesystem.hpp"
#include "boost/regex.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <fstream>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <time.h>
#include <iomanip>

using namespace agv_exploration;
using namespace actionlib;
using namespace ros;
using namespace boost;
using namespace std;

/**
 * Constructor.
 * @param n The ROS node handle.
 */
AgvExplorerTest::AgvExplorerTest(NodeHandle n)
{
    n.param("test_timeout", this->_testTimeout, TEST_TIMEOUT);
    n.param<string>("working_folder", this->_workingFolder, WORKING_FOLDER_DEFAULT);
    n.param<string>("output_folder", this->_outputFolder, OUTPUT_FOLDER_DEFAULT);
    this->retriveLaunchFiles();
    this->startTestSimulation();
}

/**
 * Destructor.
 */
AgvExplorerTest::~AgvExplorerTest()
{
    this->killAllNodes();
    std::system("pkill roslaunch");
}

/**
 * Retrieves the relevant launch files and sets them in _launchFileNames property.
 */
void AgvExplorerTest::retriveLaunchFiles()
{
    string fileName;

    // Iteration of all the subfolder in agv_exploration/launch.
    for (filesystem::recursive_directory_iterator end, dir(_workingFolder + "agv-exploration/agv_exploration_test/launch/");
        dir != end;
        ++dir)
    {
        fileName = dir->path().filename().string();

        // Check if it is a launch file 
        if (regex_match(fileName, regex("^(?!.*agv_exploration_test).*(\\.launch)$")))
        {
            _launchFileNames.push_back(fileName);
        }
    }
}

/**
 * Starts the test simulation with all the retrieved launch files.
 */
void AgvExplorerTest::startTestSimulation()
{
    // Create output folder based on timestamp and save path for map and log csv output.
    _outputTestFolderName = _outputFolder + "test_" + getCurrentTimeStamp();

    // Everyone can read, write and execute.
    mkdir(_outputTestFolderName.c_str(), S_IRWXU | S_IRWXO);

    // Create a csv file. In the end of the test this file will be parsed by a python script to create an html file.
    ofstream logFile((_outputTestFolderName + "/log.csv").c_str());
    logFile.close();

    for (int i = 0; i < _launchFileNames.size(); i++)
    {

        // Build non blocking system call
        string command = "roslaunch agv_exploration_test " + _launchFileNames[i] + " isTest:=true &";

        // Remove _nosim.launch from the name of the file
        string nameOfTheMap = _launchFileNames[i].erase((_launchFileNames[i].find(".launch")), 7);

        ROS_INFO_STREAM("Start simulation for: " << nameOfTheMap);
        std::system(command.c_str());

        SimpleActionClient<StartExplorationAction> actionClient("AgvStartExploration", true);
        actionClient.waitForServer(Duration(30));
        StartExplorationGoal goal;

        // Start the timer
        time_t start, end;
        int elapsed;
        time(&start);

        // Send the goal and wait until it is either completed, aborted or the specified timeout was exceeded.
        SimpleClientGoalState state = actionClient.sendGoalAndWait(goal, Duration(this->_testTimeout));
        time(&end);
        elapsed = difftime(end, start);
        string formattedTime = this->getFormattedTimerTime(elapsed);

        logFile.open((_outputTestFolderName + "/log.csv").c_str(), ios::out | ios::app);

        if (state == SimpleClientGoalState::ABORTED)
        {
            logFile << nameOfTheMap << ";" << formattedTime << ";"
                    << "FAIL"
                    << ";" << nameOfTheMap << "\n";
        }
        else if (state == SimpleClientGoalState::SUCCEEDED)
        {
            logFile << nameOfTheMap << ";" << formattedTime << ";"
                    << "PASS"
                    << ";" << nameOfTheMap << "\n";
        }
        else if (state == SimpleClientGoalState::PENDING || state == SimpleClientGoalState::ACTIVE)
        {
            logFile << nameOfTheMap << ";" << formattedTime << ";"
                    << "PENDING/ACTIVE"
                    << ";" << nameOfTheMap << "\n";
            actionClient.cancelAllGoals();
        }
        else
        {
            // Show error.
            ROS_WARN("Something went wrong during the simulation, %s state!", state.toString().c_str());
            logFile << nameOfTheMap << ";" << formattedTime << ";" << state.toString() << ";" << nameOfTheMap << "\n";
        }

        logFile.close();
        std::system(("rosrun map_server map_saver -f " + _outputTestFolderName + "/" + nameOfTheMap + " &").c_str());

        ROS_INFO_STREAM("Stop simulation for: " << nameOfTheMap);
        this->killAllNodes();
    }

    // Run buildlogfile.py so it converts the log.csv file to a log.html file in the output directory.
    std::system(("python " + _workingFolder +
                         "agv-exploration/agv_exploration_test/src/buildlogfile.py " 
                         + _outputTestFolderName + "/log.csv "
                         + _outputTestFolderName + "/").c_str());
    // Run checkbuildresults.py to check if there are any tests that have status different than success.
    std::system(("python " + _workingFolder +
                         "agv-exploration/agv_exploration_test/src/checkbuildresults.py "
                         + _outputTestFolderName + "/log.csv").c_str());
}

/**
 * Gets the current timestamp.
 * @return The current timestamp.
 */
string AgvExplorerTest::getCurrentTimeStamp()
{
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, 80, "%d-%m-%Y_%H%M%S", timeinfo);
    string formatted(buffer);

    return formatted;
}

/**
 * Kills all running nodes except for the test node.
 */
void AgvExplorerTest::killAllNodes()
{
    std::system("rosnode kill Stage");
    std::system("rosnode kill agv_exploration");
    std::system("rosnode kill GetMap");
    std::system("rosnode kill Mapper");
    std::system("rosnode kill Navigator");
    std::system("rosnode kill Operator");
    std::system("rosnode kill SetGoal");
    std::system("rosnode kill Explore");
    std::system("pkill map_saver");
}

/**
 * Gets the formatted timer time.
 * @param value The value of the timer to be formatted.
 * @return The formatted value.
 */
string AgvExplorerTest::getFormattedTimerTime(int value)
{
    stringstream result;
    int hour = value / 3600;
    int minutes = (value % 3600) / 60;
    int seconds = value % 60;

    result << setfill('0') << setw(2) << hour << "h "
           << setfill('0') << setw(2) << minutes << "m "
           << setfill('0') << setw(2) << seconds << "s";

    return result.str();
}
