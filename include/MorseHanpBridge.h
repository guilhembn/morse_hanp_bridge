//
// Created by gbuisan on 10/24/17.
//

#ifndef MORSE_HANP_BRIDGE_MORSEHANPBRIDGE_H
#define MORSE_HANP_BRIDGE_MORSEHANPBRIDGE_H

#include "ros/ros.h"
#include "hanp_msgs/TrackedHumans.h"
#include "hanp_msgs/TrackedSegment.h"
#include "hanp_msgs/TrackedSegmentType.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/tf.h"
#include "signal.h"
#include <tf/transform_listener.h>

#include "morse_hanp_bridge/morse_hanp_bridgeConfig.h"
#include "dynamic_reconfigure/server.h"

#define SUB_TOPIC "/human_pose"
#define PUB_TOPIC "/tracked_human_pose"
#define MARKER_PUB_TOPIC "/tracked_human_pose_markers"
#define PUB_RATE 10

typedef struct {
    bool restrain_to_robot_FOV;
    double robot_FOV_angle;
    double robot_FOV_range;
}morse_hanp_bridgeCfg;

class MorseHanpBridge{

public:
    ros::Timer publishTimer;
    ros::NodeHandle n;
    ros::Subscriber poseSubscriber;
    ros::Publisher humanPub;
    ros::Publisher markerPub;
    bool isCurrentPose;
    geometry_msgs::PoseStamped currentPose;
    bool isLastPose;
    geometry_msgs::PoseStamped lastPose;
    bool isCurrentVelocity;
    geometry_msgs::TwistStamped currentVelocity;
    bool isLastVelocity;
    geometry_msgs::TwistStamped lastVelocity;
    morse_hanp_bridgeCfg config;
    tf::TransformListener listener;

    MorseHanpBridge();
    ~MorseHanpBridge();
    void publishHumans(const ros::TimerEvent& e);
    void fillVelocity (hanp_msgs::TrackedSegment &segment);
    void fillAcceleration (hanp_msgs::TrackedSegment &segment);
    void humanPoseCallback(const geometry_msgs::PoseStamped& poseMsg);
    void dynConfigCallback(morse_hanp_bridge::morse_hanp_bridgeConfig &config, uint32_t level);
    void fillAndSendHumans();



};

#endif //MORSE_HANP_BRIDGE_MORSEHANPBRIDGE_H
