#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <eigen_conversions/eigen_msg.h>

#define IMAGE_TOPIC "/usb_cam/image_raw"

//const cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
//const cv::Mat distCoeffs   = cv::Mat::eye(4,4, CV_64F);
const cv::Mat cameraMatrix = (cv::Mat_<float>(3,3) << 6.5746697944293521e+002, 0., 3.1950000000000000e+002, 0., 6.5746697944293521e+002, 2.3950000000000000e+002, 0., 0., 1.);
const std::vector<float> distCoeffs = {-4.1802327176423804e-001, 5.0715244063187526e-001, 0., 0., -5.7843597214487474e-001};

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
cv::Ptr<cv::aruco::Board> board;// = cv::aruco::GridBoard::create(5, 7, 0.04, 0.01, dictionary);

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
geometry_msgs::Quaternion rpyToQuat(double r, double p, double y) {
    geometry_msgs::Quaternion ret;

	// Abbreviations for the various angular functions
	double cy = cos(y * 0.5);
	double sy = sin(y * 0.5);
	double cr = cos(r * 0.5);
	double sr = sin(r * 0.5);
	double cp = cos(p * 0.5);
	double sp = sin(p * 0.5);

	ret.w = cy * cr * cp + sy * sr * sp;
	ret.x = cy * sr * cp - sy * cr * sp;
	ret.y = cy * cr * sp + sy * sr * cp;
	ret.z = sy * cr * cp - cy * sr * sp;
	return ret;
}

struct ArucoMarker {
    int id;
    geometry_msgs::Pose pose;
    std::vector<cv::Point3f> corners;
};

std::vector<ArucoMarker> markers;
void loadBoardMarkers() {
    double marker_size;
    ros::param::get("~marker_size", marker_size);

    XmlRpc::XmlRpcValue marker_list;
    if(ros::param::get("~markers", marker_list)) {
        for(int i = 0; i < marker_list.size(); i++) {
            XmlRpc::XmlRpcValue &marker = marker_list[i]["marker"];

            markers.emplace_back();
            ArucoMarker &newMarker = markers.back();

            geometry_msgs::Point position;
            position.x = static_cast<double>(marker["position"]["x"]);
            position.y = static_cast<double>(marker["position"]["y"]);
            position.z = static_cast<double>(marker["position"]["z"]);

            Eigen::Vector3d center(position.x, position.y, position.z);
            for(Eigen::Vector3d offset : 
                {
                    Eigen::Vector3d(-marker_size/2.0, marker_size/2.0, 0),
                    Eigen::Vector3d(marker_size/2.0, marker_size/2.0, 0),
                    Eigen::Vector3d(marker_size/2.0, -marker_size/2.0, 0),
                    Eigen::Vector3d(-marker_size/2.0, -marker_size/2.0, 0),
                })
            {
                cv::Point3f corner(center.x(), center.y(), center.z());
                cv::Point3f cvOffset(offset.x(), offset.y(), offset.z());
                newMarker.corners.emplace_back(corner + cvOffset);
            }

            double r = static_cast<double>(marker["orientation"]["r"]);
            double p = static_cast<double>(marker["orientation"]["p"]);
            double y = static_cast<double>(marker["orientation"]["y"]);

            newMarker.id = static_cast<int>(marker["id"]);

            newMarker.pose.orientation = rpyToQuat(r, p, y);
            newMarker.pose.position = position;
        }
    }
    std::vector<int> ids;
    std::vector<std::vector<cv::Point3f>> corners;

    ROS_INFO("Creating board...");
    for(const auto &marker : markers) {
        ids.emplace_back(marker.id);
        corners.emplace_back(marker.corners);
        ROS_INFO_STREAM(marker.id << ":" << marker.corners);
    }

    board = cv::aruco::Board::create(corners, dictionary, ids);
}

image_transport::Publisher pub;
ros::Publisher posePub;
void colorCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
    cv::Mat &color = image->image;

    cv::Mat imageCopy;
    color.copyTo(imageCopy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(color, dictionary, corners, ids);
    // if at least one marker detected
    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        cv::Vec3d rvec, tvec;
        int valid = estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvec, tvec);
        // if at least one board marker detected
        if(valid > 0) {
            cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1);

			cv::Mat rotationMatrix;
			cv::Rodrigues(rvec, rotationMatrix);

			Eigen::Matrix3d eigenRotationMatrix;
			cv::cv2eigen(rotationMatrix, eigenRotationMatrix);

            Eigen::Affine3d boardPose;
            boardPose = eigenRotationMatrix;
            boardPose *= Eigen::Translation3d(tvec[0], tvec[1], tvec[2]);

            Eigen::Affine3d robotPose = boardPose.inverse();

            geometry_msgs::Pose robotPoseMsg;
			tf::poseEigenToMsg(robotPose, robotPoseMsg);

            geometry_msgs::PoseWithCovarianceStamped outputPose;
            outputPose.header.stamp = msg->header.stamp;
            outputPose.header.frame_id = "map";
            outputPose.pose.pose = robotPoseMsg;

            posePub.publish(outputPose);
        }
    }

    sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCopy).toImageMsg();
    pub.publish(debug_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "board_tracker");
    ros::NodeHandle node;

    loadBoardMarkers();

    image_transport::ImageTransport it(node);
    image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, colorCallback);
    pub = it.advertise("debug", 1);

    posePub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("aruco_pose", 5);

    ros::spin();
}

