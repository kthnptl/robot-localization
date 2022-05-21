#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h" // present under /devel/include/ball_chaser/<*.h>

ros::Publisher motor_command_publisher; // Global ROS::Publisher motor commands



/* 
 *
 *  This function checks and clamps the joint angles to a safe zone 
 *
 */
std::vector<float> clamp_at_boundaries(float requested_linearX, float requested_angularZ)
{
    float clamped_linearX  = requested_linearX;
    float clamped_angularZ = requested_angularZ;

    float   min_linearX, 
            max_linearX, 
            min_angularZ, 
            max_angularZ;

    ros::NodeHandle n2;

    std::string node_name = ros::this_node::getName();

    // Get joints min and max parameters
    n2.getParam(node_name + "/min_linear_x", min_linearX);
    n2.getParam(node_name + "/max_linear_x", max_linearX);
    n2.getParam(node_name + "/min_angular_z", min_angularZ);
    n2.getParam(node_name + "/max_angular_z", max_angularZ);

    // Check if linear_x falls in the safe zone, otherwise clamp it
    if (requested_linearX < min_linearX || requested_linearX > max_linearX) {
        clamped_linearX = std::min(std::max(requested_linearX, min_linearX), max_linearX);
        ROS_WARN("linear_x is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_linearX, max_linearX, clamped_linearX);
    }
    
    // Check if anugular_z falls in the safe zone, otherwise clamp it
    if (requested_angularZ < min_angularZ || requested_angularZ > max_angularZ) {
        clamped_angularZ = std::min(std::max(requested_angularZ, min_angularZ), max_angularZ);
        ROS_WARN("angular_z is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_angularZ, max_angularZ, clamped_angularZ);
    }

    // Store clamped wheel joint angles in a clamped_velocities vector
    std::vector<float> clamped_velocities = { clamped_linearX, clamped_angularZ };

    return clamped_velocities;
}



/*
 *
 *  Created a "handle_drive_request" callback function that executes whenever a drive_bot service is requested
 *  This function should publish the requested linear x and angular velocities to the robot wheel joints
 *  After publishing the requested velocities, a message feedback is returned with the requested wheel velocities
 *
 */
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("DriveToTargetRequest received - lin_x:%1.2f, ang_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

    /*
    // Check if the requested velocities are in safe zone, otherwise clamp them
    std::vector<float> velocities = clamp_at_boundaries(req.linear_x, req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist 
    // and obtain linear_x and angular_z from clamp_at_boundaries function
    geometry_msgs::Twist motor_command;
    motor_command.linear.x  = velocities[0];
    motor_command.angular.z = velocities[1];
    */
    // TODO: remove multi-line comment from above
    geometry_msgs::Twist motor_command;
    motor_command.linear.x  = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);

    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();

    // Return a response message
    // TODO: 
    //res.msg_feedback = "Wheel joint angles set - linear_x: " + std::to_string(velocites[0]) + " , angular_z: " + std::to_string(velocities[1]);
    res.msg_feedback = "Wheel joint angles set - linear_x: " + std::to_string(req.linear_x) + " , angular_z: " + std::to_string(req.angular_z);

    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}



/*
 *
 * MAIN function
 *
 */
int main(int argc, char** argv)
{
    // Initialize a ROS node and Create a ROS NodeHandle object
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    // geometry_msgs::Twist information -> http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}

