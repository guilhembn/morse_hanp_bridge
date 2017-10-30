#include "MorseHanpBridge.h"

MorseHanpBridge::~MorseHanpBridge() = default;

MorseHanpBridge::MorseHanpBridge() {
    isCurrentPose = false;
    isLastPose = false;
    isCurrentVelocity = false;
    isLastVelocity = false;
    humanPub = n.advertise<hanp_msgs::TrackedHumans>(PUB_TOPIC, 1);
    markerPub = n.advertise<visualization_msgs::MarkerArray>(MARKER_PUB_TOPIC, 1);
}

void MorseHanpBridge::publishHumans(const ros::TimerEvent &e) {
    if (isLastPose) {
        auto now = ros::Time::now();
        hanp_msgs::TrackedHumans trackedHumansMsg;
        hanp_msgs::TrackedHuman trackedHuman;
        hanp_msgs::TrackedSegment trackedSegment;
        visualization_msgs::MarkerArray humans_markers;
        trackedSegment.type = hanp_msgs::TrackedSegmentType::TORSO;
        trackedSegment.pose.pose = currentPose.pose;
        fillVelocity(trackedSegment);
        if (isLastVelocity) {
            fillAcceleration(trackedSegment);
            trackedHuman.track_id = 3;
            trackedHuman.segments.push_back(trackedSegment);
            trackedHumansMsg.humans.push_back(trackedHuman);
            trackedHumansMsg.header.frame_id = "optitrack";

            visualization_msgs::Marker human_arrow, human_cylinder;

            human_arrow.header.stamp = now;
            human_arrow.header.frame_id = "optitrack";
            human_arrow.type = visualization_msgs::Marker::ARROW;
            human_arrow.action = visualization_msgs::Marker::MODIFY;
            human_arrow.id = 103;
            human_arrow.pose.position.x = trackedSegment.pose.pose.position.x;
            human_arrow.pose.position.y = trackedSegment.pose.pose.position.y;
            human_arrow.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(trackedSegment.pose.pose.orientation));
            human_arrow.scale.x = 0.25 * 2.0;
            human_arrow.scale.y = 0.1;
            human_arrow.scale.z = 0.1;
            human_arrow.color.a = 1.0;
            human_arrow.color.r = 0;
            human_arrow.color.g = 150;
            human_arrow.color.b = 200;
            human_arrow.lifetime = ros::Duration(4.0);

            human_cylinder.header.stamp = now;
            human_cylinder.header.frame_id = "optitrack";
            human_cylinder.type = visualization_msgs::Marker::CYLINDER;
            human_cylinder.action = visualization_msgs::Marker::MODIFY;
            human_cylinder.id = 3;
            human_cylinder.pose.position.x = trackedSegment.pose.pose.position.x;
            human_cylinder.pose.position.y = trackedSegment.pose.pose.position.y;
            human_cylinder.pose.position.z += (1.5 / 2);
            // human_cylinder.pose.orientation = body_segment.pose.orientation;
            human_cylinder.scale.x = 0.25 * 2;
            human_cylinder.scale.y = 0.25 * 2;
            human_cylinder.scale.z = 1.5;
            human_cylinder.color.a = 1.0;
            human_cylinder.color.r = 0;
            human_cylinder.color.g = 150;
            human_cylinder.color.b = 200;
            human_cylinder.lifetime = ros::Duration(4.0);

            humans_markers.markers.push_back(human_arrow);
            humans_markers.markers.push_back(human_cylinder);

            trackedHumansMsg.header.stamp = now;

            humanPub.publish(trackedHumansMsg);
            markerPub.publish(humans_markers);
        }
    }
}

void MorseHanpBridge::fillVelocity(hanp_msgs::TrackedSegment &segment) {
    double dt = (ros::Time(currentPose.header.stamp.sec, currentPose.header.stamp.nsec) -
            ros::Time(lastPose.header.stamp.sec, lastPose.header.stamp.nsec)).toSec();
    tf::Vector3 position_diff(currentPose.pose.position.x - lastPose.pose.position.x,
                              currentPose.pose.position.y - lastPose.pose.position.y,
                              currentPose.pose.position.z - lastPose.pose.position.z);

    double roll_diff, pitch_diff, yaw_diff;
    tf::Matrix3x3(tf::Quaternion(currentPose.pose.orientation.x,
                                 currentPose.pose.orientation.y,
                                 currentPose.pose.orientation.z,
                                 currentPose.pose.orientation.w)
                          .inverse() * tf::Quaternion(lastPose.pose.orientation.x,
                                                      lastPose.pose.orientation.y,
                                                      lastPose.pose.orientation.z,
                                                      lastPose.pose.orientation.w))
            .getRPY(roll_diff, pitch_diff, yaw_diff);
    lastVelocity = currentVelocity;
    isLastVelocity = isCurrentVelocity;
    currentVelocity.header.stamp = currentPose.header.stamp;
    currentVelocity.twist.linear.x = position_diff.x() / dt;
    currentVelocity.twist.linear.y = position_diff.y() / dt;
    currentVelocity.twist.linear.z = position_diff.z() / dt;
    currentVelocity.twist.angular.x = roll_diff / dt;
    currentVelocity.twist.angular.y = pitch_diff / dt;
    currentVelocity.twist.angular.z = yaw_diff / dt;
    isCurrentVelocity = true;

    segment.twist.twist = currentVelocity.twist;

}

void MorseHanpBridge::fillAcceleration(hanp_msgs::TrackedSegment &segment){
    double dt = (ros::Time(currentVelocity.header.stamp.sec, currentPose.header.stamp.nsec) -
                 ros::Time(lastVelocity.header.stamp.sec, lastPose.header.stamp.nsec)).toSec();
    tf::Vector3 lvelocity_diff(currentVelocity.twist.linear.x  - lastVelocity.twist.linear.x,
                               currentVelocity.twist.linear.y  - lastVelocity.twist.linear.y,
                               currentVelocity.twist.linear.z  - lastVelocity.twist.linear.z);
    tf::Vector3 avelocity_diff(currentVelocity.twist.angular.x - lastVelocity.twist.angular.x,
                               currentVelocity.twist.angular.y - lastVelocity.twist.angular.y,
                               currentVelocity.twist.angular.z - lastVelocity.twist.angular.z);
    segment.accel.accel.linear.x = lvelocity_diff.x() / dt;
    segment.accel.accel.linear.y = lvelocity_diff.y() / dt;
    segment.accel.accel.linear.z = lvelocity_diff.z() / dt;
    segment.accel.accel.angular.x = avelocity_diff.x() / dt;
    segment.accel.accel.angular.y = avelocity_diff.y() / dt;
    segment.accel.accel.angular.z = avelocity_diff.z() / dt;
}

void MorseHanpBridge::humanPoseCallback(const geometry_msgs::PoseStamped &poseMsg) {
    lastPose = currentPose;
    isLastPose = isCurrentPose;
    currentPose = poseMsg;
    isCurrentPose = true;
}

void sigintHandler(int sig){
    ROS_DEBUG_STREAM("node will now shutdown");
    // the default sigint handler, it calls shutdown() on node
    ros::shutdown();
    exit(sig);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "morse_hanp_bridge");
    signal(SIGINT, sigintHandler);
    MorseHanpBridge mhb;
    mhb.publishTimer = mhb.n.createTimer(ros::Duration(0.1), &MorseHanpBridge::publishHumans, &mhb);
    mhb.poseSubscriber = mhb.n.subscribe(SUB_TOPIC, 1, &MorseHanpBridge::humanPoseCallback, &mhb);
    ROS_INFO("Morse Hanp Bridge launched");
    ros::spin();

    return 0;
}