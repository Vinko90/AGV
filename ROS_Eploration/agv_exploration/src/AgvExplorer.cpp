#include <agv_exploration/AgvExplorer.h>
#include <actionlib/client/simple_action_client.h>
#include <nav2d_navigator/GetFirstMapAction.h>
#include <nav2d_navigator/GetFirstMapGoal.h>
#include <nav2d_navigator/ExploreAction.h>
#include <nav2d_navigator/ExploreGoal.h>
#include <nav2d_operator/cmd.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_listener.h>
#include <agv_exploration/StartExplorationGoal.h>

using namespace agv_exploration;
using namespace actionlib;
using namespace tf;
using namespace nav2d_navigator;
using namespace nav2d_operator;
using namespace ros;

/**
 * Constructor.
 * @param n The ROS node handle.
 */
AgvExplorer::AgvExplorer(NodeHandle n)
{
    // Get parameters from parameters server.
    n.param("wait_for_server_duration", this->waitForServerDuration_, WAIT_FOR_SERVER_DURATION);
    n.param(
        "wait_for_get_first_map_duration",
        this->waitForGetFirstMapDuration_,
        WAIT_FOR_GET_FIRST_MAP_DURATION);
    n.param(
        "unstuck_after_not_moving_for_seconds",
        this->unstuckAfterNotMovingForSeconds_,
        UNSTUCK_AFTER_NOT_MOVING_FOR_SECONDS);
    n.param(
        "try_unstuck_seconds_per_direction",
        this->tryUnstuckSecondsPerDirection_,
        TRY_UNSTUCK_SECONDS_PER_DIRECTION);
    n.param("max_retries", this->maxRetries_, MAX_RETRIES);

    this->cmdPublisher_ = n.advertise<cmd>("cmd", 1000);
    this->listener_ = new TransformListener();

    this->startExplorationServer_ =
        new SimpleActionServer<StartExplorationAction>(n, "AgvStartExploration", false);
    this->startExplorationServer_->registerGoalCallback(boost::bind(&AgvExplorer::goalCallback, this));
    this->startExplorationServer_->start();

    spin();
}

/**
 * Destructor.
 */
AgvExplorer::~AgvExplorer()
{
    this->startExplorationServer_->shutdown();
    delete this->listener_;
    delete this->startExplorationServer_;
}

/**
 * Gets the initial map. Will make the AGV move forward and then do a 360 degrees rotation.
 */
void AgvExplorer::getFirstMap()
{
    SimpleActionClient<GetFirstMapAction> getFirstMapClient("GetFirstMap", true);

    ROS_INFO("Waiting for Nav2d GetFirstMap server");
    getFirstMapClient.waitForServer(Duration(this->waitForServerDuration_));

    GetFirstMapGoal getFirstMapGoal;
    ROS_INFO("Started mapping");
    getFirstMapClient.sendGoalAndWait(getFirstMapGoal, Duration(this->waitForGetFirstMapDuration_));
    getFirstMapClient.cancelAllGoals();
    ROS_INFO("Mapping completed");
}

/**
 * Starts exploration of the environment. The function handles stuck issues and returns when the exploration is
 * completed.
 */
void AgvExplorer::startExploration()
{
    SimpleActionClient<ExploreAction> *exploreClient = new SimpleActionClient<ExploreAction>("Explore", true);

    ROS_INFO("Waiting for Nav2d Explore server");
    exploreClient->waitForServer(Duration(this->waitForServerDuration_));

    ExploreGoal exploreGoal;
    ROS_INFO("Started exploration");
    exploreClient->sendGoal(exploreGoal);

    Duration d(this->unstuckAfterNotMovingForSeconds_);
    StampedTransform previousAgvTransform, currentAgvTransform = getAgvTransform();
    short restartingAttemptCounter = 1;

    // Wait initially.
    d.sleep();
    do
    {
        // Check if the exploration goal should be preempted and stop exploration.
        if (this->startExplorationServer_->isPreemptRequested())
        {
            exploreClient->cancelAllGoals();
            this->startExplorationServer_->setPreempted();
            ROS_WARN("Exploration cancelled.");
            break;
        }

        // Aborted or preempted state can occur when the exploration crashes because of not being able to find
        // path to the goal or because of the robot being stuck.
        SimpleClientGoalState currentActionState = exploreClient->getState();
        if (currentActionState == SimpleClientGoalState::ABORTED ||
            currentActionState == SimpleClientGoalState::PREEMPTED)
        {
            if (restartingAttemptCounter > this->maxRetries_)
            {
                ROS_ERROR(
                    "Exploration crashed critically. Unable to recover after %d retries.", this->maxRetries_);
                this->startExplorationServer_->setAborted();
                break;
            }

            tryUnstuck();

            // If the exploration was aborted, restart it.
            ROS_WARN("Exploration was aborted. Restarting %d time...", restartingAttemptCounter);
            exploreClient = new SimpleActionClient<ExploreAction>("/Explore", true);
            exploreClient->sendGoal(exploreGoal);

            // Increment restart counter.
            restartingAttemptCounter++;
        }
        else
        {
            // If the restart was successful, then reset the counter.
            restartingAttemptCounter = 1;
        }

        // The AGV can also get stuck without the exploration being aborted or preempted, therefore we check
        // again here.
        currentAgvTransform = getAgvTransform();
        if (!hasAgvMoved(previousAgvTransform, currentAgvTransform))
        {
            ROS_WARN("AGV is stuck");
            tryUnstuck();
        }

        // Set the current transform, pose and yaw as previous and then wait.
        previousAgvTransform = currentAgvTransform;
        d.sleep();
    } while (exploreClient->getState() != SimpleClientGoalState::SUCCEEDED);

    if (this->startExplorationServer_->isActive())
    {
        // Set succeeded only when the exploration was not cancelled.
        ROS_INFO("Exploration completed");
        this->startExplorationServer_->setSucceeded();
    }

    delete exploreClient;
}

/**
 * Gets the current AGV transform used to determine its position and orientation.
 * @return A StampedTransform representing the AGV's current position and orientation.
 */
StampedTransform AgvExplorer::getAgvTransform()
{
    StampedTransform transform;
    try
    {
        // Get the latest available transform.
        this->listener_->lookupTransform("map", "base_link", Time(0), transform);
    }
    catch (TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    return transform;
}

/**
 * Tries to unstuck the AGV by moving in different direction for specified by tryUnstuckSecondsPerDirection amount of
 * seconds.
 * @param unstuckType The unstuck direction to which the AGV should move.
 */
void AgvExplorer::tryUnstuck(short unstuckType)
{
    StampedTransform previousAgvTransform = getAgvTransform();
    cmd cmd;
    ros::Duration d(this->tryUnstuckSecondsPerDirection_);

    // Turn [-1.0 .. 1.0]: -1(rotate left); 0(straight); 1(rotate right)
    // Velocity  [-1.0 .. 1.0]: -1(full speed back); 0(stop); 1(full speed ahead)
    // Mode: 0(Avoid obstacles); 1(Stop at obstacles)
    switch (unstuckType)
    {
    case UNSTUCK_FORWARD:
        cmd.Mode = 0;
        cmd.Turn = 0;
        cmd.Velocity = 1;
        ROS_INFO("Trying to unstuck by moving forward...");
        break;
    case UNSTUCK_BACKWARDS:
        cmd.Mode = 0;
        cmd.Turn = 0;
        cmd.Velocity = -1;
        ROS_INFO("Trying to unstuck by moving backwards...");
        break;
    case UNSTUCK_LEFT:
        cmd.Mode = 0;
        cmd.Turn = -1;
        cmd.Velocity = 1;
        ROS_INFO("Trying to unstuck by moving left...");
        break;
    case UNSTUCK_RIGHT:
        cmd.Mode = 0;
        cmd.Turn = 1;
        cmd.Velocity = 1;
        ROS_INFO("Trying to unstuck by moving right...");
        break;
    }

    // Move for 1 second in the specified direction and then stop.
    this->cmdPublisher_.publish(cmd);
    d.sleep();

    cmd.Velocity = 0;
    this->cmdPublisher_.publish(cmd);

    // If the AGV has not changed its position or orientation it means that unstucking in the current direction was not
    // successful. We increase the unstuck type if it is less than the maximum allowed value and retry.
    if (!hasAgvMoved(previousAgvTransform, getAgvTransform()) && unstuckType < UNSTUCK_RIGHT)
    {
        tryUnstuck(++unstuckType);
    }
}

/**
 * Executed on receiving a new goal.
 */
void AgvExplorer::goalCallback()
{
    this->startExplorationServer_->acceptNewGoal();
    ROS_INFO("Goal accepted.");
    this->getFirstMap();

    // Run the exploration loop on a separate thread so it does not block the communication with the
    // exploration server.
    boost::thread t1(boost::bind(&AgvExplorer::startExploration, this));
}

/**
 * Checks whether the AGV has changed its position or orientation. By using the previous and current AGV
 * transformations.
 * @param previousAgvTransform The last AGV transformation.
 * @param currentAgvTransform The current AGV transformation.
 * @return True if the AGV has changed its position or orientation, otherwise false.
 */
bool AgvExplorer::hasAgvMoved(StampedTransform previousAgvTransform, StampedTransform currentAgvTransform)
{
    Vector3 previousAgvPose, currentAgvPose;
    double previousAgvYaw, currentAgvYaw;

    previousAgvPose = previousAgvTransform.getOrigin();
    currentAgvPose = currentAgvTransform.getOrigin();

    previousAgvYaw = getYaw(previousAgvTransform.getRotation());
    currentAgvYaw = getYaw(currentAgvTransform.getRotation());

    // If X, Y and Yaw of the AGV are the same, then it has not moved.
    return !(previousAgvPose.getX() == currentAgvPose.getX() &&
             previousAgvPose.getY() == currentAgvPose.getY() &&
             previousAgvYaw == currentAgvYaw);
}