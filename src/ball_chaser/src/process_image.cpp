#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
// 0.5  0.0 -> forward
// 0.0  0.5 -> left
// 0.0 -0.5 -> right
// 0.0  0.0 -> stop
void drive_robot(float lin_x, float ang_z) {
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget msg;
    msg.request.angular_z = ang_z;
    msg.request.linear_x = lin_x;

    if (!client.call(msg)) ROS_ERROR("Failed to call service drive_bot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img) {

    constexpr int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    int x = 0;
    int y = 0;
    for (y = 0; y < img.height; ++y) {
        bool found = false;
        for (x = 0; x < img.step; ++x) {
            if (img.data[x + y * img.step] == white_pixel) {
                ROS_INFO("Found ball at x:%d, y:%d!", x, y);
                found = true;
                break;
            }
        }
        if (found) {
            break;
        }
    }

    if (x == img.step && y == img.height) {
        // send stop
        ROS_INFO("Send stop!");
        drive_robot(0.0, 0.0);
        return;
    }

    const int left_border = img.step / 3;
    const int right_border = img.step / 3 * 2;

    if (x < left_border) {
        // drive left
        ROS_INFO("Send left");
        drive_robot(0.0, 0.5);
        return;
    }

    if (x >= right_border) {
        // drive right
        ROS_INFO("Send right");
        drive_robot(0.0, -0.5);
        return;
    }

    if (x > left_border && x < right_border) {
        // drive forward
        ROS_INFO("Send forward");
        drive_robot(0.5, 0.0);
        return;
    }
}

int main(int argc, char **argv) {
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}