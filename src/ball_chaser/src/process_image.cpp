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

    assert(img.step % 3 == 0);

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    int counter_left = 0;
    int counter_right = 0;
    int counter_middle = 0;

    for (int y = 0; y < img.height; ++y) {
        for (int stepper = 0; stepper < img.step; stepper += 3) {

            // index in the array
            const int index = stepper + y * img.step;

            if (img.data[index] == white_pixel && img.data[index + 1] == white_pixel &&
                img.data[index + 2] == white_pixel) {
                // left third of the image
                const int left_border = img.step / 3;
                // right third of the image
                const int right_border = img.step / 3 * 2;

                if (stepper < left_border) {
                    ++counter_left;
                } else if (stepper >= right_border) {
                    ++counter_right;
                } else {
                    ++counter_middle;
                }
            }
        }
    }

    // there is no ball
    if (counter_left == 0 && counter_middle == 0 && counter_right == 0) {
        // send stop
        ROS_INFO("Send stop!");
        drive_robot(0.0, 0.0);
        return;
    }

    // more pixels on the left side
    if (counter_left > counter_middle && counter_left > counter_right) {
        // drive left
        ROS_INFO("Send left");
        drive_robot(0.0, 0.5);
        return;
    }

    if (counter_right > counter_left && counter_right > counter_middle) {
        // drive right
        ROS_INFO("Send right");
        drive_robot(0.0, -0.5);
        return;
    }

    // drive forward
    ROS_INFO("Send forward");
    drive_robot(0.5, 0.0);
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