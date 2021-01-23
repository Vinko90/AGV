#ifndef AGVEXPLORER_H_
#define AGVEXPLORER_H_

#include <ros/ros.h>
#include <agv_exploration/StartExplorationAction.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <boost/thread/thread.hpp>

#define WAIT_FOR_SERVER_DURATION 30
#define WAIT_FOR_GET_FIRST_MAP_DURATION 30
#define UNSTUCK_AFTER_NOT_MOVING_FOR_SECONDS 2
#define TRY_UNSTUCK_SECONDS_PER_DIRECTION 1
#define MAX_RETRIES 10

class AgvExplorer {
    public:
        /**
         * Constructor.
         * @param n The ROS node handle.
         */
        AgvExplorer(ros::NodeHandle n);

        /**
         * Destructor.
         */
        ~AgvExplorer();

    private:
        static const short UNSTUCK_FORWARD = 0,
                           UNSTUCK_BACKWARDS = 1,
                           UNSTUCK_LEFT = 2,
                           UNSTUCK_RIGHT = 3;

            /**
             * The amount of seconds for which the node should wait for a server to become available before crashing.
             * Setting it 0 would mean that the node will wait for an infinite period of time. The value can be set by
             * setting the "wait_for_server_duration" ROS param. By default the value is set to 30.
             */
        int waitForServerDuration_,

            /**
             * The amount of seconds for which the node should wait before GetFirstMap goal is completed. This is done
             * such that if the AGV gets stuck during that task, the node is able to continue and potentially unstuck
             * the AGV. The value can be set by setting the "wait_for_get_first_map_duration" ROS param. By default the
             * value is set to 30.
             */
            waitForGetFirstMapDuration_,

            /**
             * The amount of seconds for which the node should wait before checking whether the AGV has moved. If after
             * that amount of seconds has passed and the AGV has not moved, then the unstuck sequence will be executed.
             * The value can be set by setting the "unstuck_after_not_moving_for_seconds" ROS param. By default the
             * value is set to 2.
             */
            unstuckAfterNotMovingForSeconds_,

            /**
             * The amount of seconds for which the unstuck sequence should try to move for each direction. The value can
             * be set by setting the "try_unstuck_seconds_per_direction" ROS param. By default the value is set to 1. 
             */
            tryUnstuckSecondsPerDirection_,

            /**
             * The maximum amount of times the node should try to restart the exploration in case of a failure. If after
             * that many retries, the exploration still fails, then the exploration goal is aborted. The value can be
             * set by setting the "max_retries" ROS param. By default the value is set to 10.
             */
            maxRetries_;

        /**
         * The movement command publisher used for manually making the AGV move. Need for the unstuck sequence.
         */
        ros::Publisher cmdPublisher_;

        /**
         * The transform listener used to get the AGV's position and orientation.
         */
        tf::TransformListener* listener_;

        /**
         * The actionlib server providing an interface for starting exploration. The server listens to the
         * AgvStartExploration topic.
         */
        actionlib::SimpleActionServer<agv_exploration::StartExplorationAction>* startExplorationServer_;

        /**
         * Gets the initial map. Will make the AGV move forward and then do a 360 degrees rotation.
         */
        void getFirstMap();

        /**
         * Starts exploration of the environment. The function handles stuck issues and returns when the exploration is
         * completed.
         */
        void startExploration();

        /**
         * Tries to unstuck the AGV by moving in different direction for specified by tryUnstuckSecondsPerDirection
         * amount of seconds.
         * @param unstuckType The unstuck direction to which the AGV should move.
         */
        void tryUnstuck(short unstuckType = UNSTUCK_FORWARD);

        /**
         * Executed on receiving a new goal.
         */
        void goalCallback();

        /**
         * Checks whether the AGV has changed its position or orientation. By using the previous and current AGV
         * transformations.
         * @param previousAgvTransform The last AGV transformation.
         * @param currentAgvTransform The current AGV transformation.
         * @return True if the AGV has changed its position or orientation, otherwise false.
         */
        bool hasAgvMoved(tf::StampedTransform previousAgvTransform, tf::StampedTransform currentAgvTransform);

        /**
         * Gets the current AGV transform used to determine its position and orientation.
         * @return A StampedTransform representing the AGV's current position and orientation.
         */
        tf::StampedTransform getAgvTransform();
};

#endif // AGVEXPLORER_H_