/**
 * Soccerball Tracking
 *
 * Created by Ryan Owens
 *
 * 01/28/2018
 *
 *
                # # # #
              __#_#_#_#__
             {_` ` ` ` `_}
            _{_._._._._._}_
           {_  H A P P Y  _}
          _{_._._._._._._._}_
         {_ B I R T H D A Y _}
     .---{_._._._._._._._._._}---.
    (   `"""""""""""""""""""""`   )
     `~~~~~~~~~~~~~~~~~~~~~~~~~~~`
 *
 */
#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/console.h>
#include <vector>

// Functions
std::vector<geometry_msgs::PointStamped> findBallsOfColor(int color_bounds[][3], cv::Mat frame);
// Globals
int red_bounds[2][3] = {
        {0,0,0},
        {1,1,1}
};

int blue_bounds[2][3] = {
        {0,0,0},
        {1,1,1}
};

int green_bounds[2][3] = {
        {0,0,0},
        {1,1,1}
};

/**
 * Soccer Ball Tracker
 * @param  argc paramater count
 * @param  argv paramaters
 * @return      exit status
 */
int main(int argc, char ** argv) {
        // Init ROS
        ros::init(argc, argv, "soccerball_tracking");
        // Get Node Handle
        ros::NodeHandle n;
        // Setup Publishers
        ros::Publisher red_sockerball_pub = n.advertise<geometry_msgs::PointStamped>("socerball/red", 1000);
        ros::Publisher blue_sockerball_pub = n.advertise<geometry_msgs::PointStamped>("socerball/blue", 1000);
        ros::Publisher green_sockerball_pub = n.advertise<geometry_msgs::PointStamped>("socerball/green", 1000);
        // Set the loop rate
        ros::Rate loop_rate(10);
        // Open Camera
        // TODO: Use Camera Feed from ROS?
        cv::VideoCapture camera(0); // TODO: Camera ID?
        if(!camera.isOpened()) {
                ROS_DEBUG("Unable to open camera");
                ros::shutdown();
                return -1;
        }
        // Publish
        cv::Mat frame;
        std::vector<geometry_msgs::PointStamped> found;
        while(ros::ok()) {
                // Get image from camera
                camera >> frame;
                // Find centers of soccerballs with current color
                found = findBallsOfColor(red_bounds, frame);
                // Publish the Points
                for (auto &point : found) {
                        red_sockerball_pub.publish(point);
                }

                // Find centers of soccerballs with current color
                found = findBallsOfColor(blue_bounds, frame);
                // Publish the Points
                for (auto &point : found) {
                        blue_sockerball_pub.publish(point);
                }

                // Find centers of soccerballs with current color
                found = findBallsOfColor(green_bounds, frame);
                // Publish the Points
                for (auto &point : found) {
                        green_sockerball_pub.publish(point);
                }

                ros::spinOnce();
                loop_rate.sleep();
        }
        return 0;
}

/**
 * Find Balls Of Color
 * @param color_bounds array of upper and lower color bounds
 * @param frame        CV Matrix frame from camera
 * @return             Vector of found points
 */
std::vector<geometry_msgs::PointStamped> findBallsOfColor(int color_bounds[][3], cv::Mat frame) {
        std::vector<geometry_msgs::PointStamped> found;
        cv::Mat frame_threshold, what_we_want;
        if(!frame.empty()) {
                cv::inRange(frame, cv::Vec3b(color_bounds[0][0], color_bounds[0][1], color_bounds[0][2]), cv::Vec3b(color_bounds[1][0], color_bounds[1][1], color_bounds[1][2]), frame_threshold);
                std::vector<std::vector<cv::Point> > points;
                // https://docs.opencv.org/3.3.1/d4/d73/tutorial_py_contours_begin.html
                cv::findContours(what_we_want, points, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);

                std::vector<cv::Moments> moments(points.size());
                for(std::vector<cv::Moments>::size_type i = 0; i < points.size(); ++i) {
                        moments[i] = cv::moments(points[i], false);
                }

                for(auto &moment : moments) {
                        geometry_msgs::PointStamped point;
                        point.point.x = moment.m10 / moment.m00;
                        point.point.y = moment.m01 / moment.m00;
                        // Area of Contour
                        point.point.z = moment.m00;
                        point.header.stamp = ros::Time::now();
                        found.push_back(point);
                }

        }

        return found;
}
