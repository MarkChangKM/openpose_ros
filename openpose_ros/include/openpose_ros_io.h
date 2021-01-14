#ifndef _OPENPOSE_ROS_IO
#define _OPENPOSE_ROS_IO

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  // Video write

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>


#include <openpose_ros_msgs/BoundingBox.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>
#include <openpose_ros_msgs/OpenPoseHuman3D.h>
#include <openpose_ros_msgs/OpenPoseHumanLink3D.h>
#include <openpose_ros_msgs/OpenPoseHumanJoint3D.h>
#include <openpose_ros_msgs/OpenPoseHumanList3D.h>
#include <openpose_ros_msgs/PointWithProb3D.h>

#include <openpose.h>
#include <openpose_flags.h>

// OpenPose dependencies
#include <openpose/headers.hpp>

#define PI 3.14159265

const int body_joint_list[25][3] = {
{0,1,8},
{0,1,2},
{0,1,5},
{2,1,5},
{2,1,8},
{5,1,8},
{1,2,3},
{2,3,4},
{1,5,6},
{5,6,7},
{1,8,9},
{1,8,12},
{9,8,12},
{8,9,10},
{10,11,22},
{11,22,23},
{8,12,13},
{12,13,14},
{13,14,19},
{14,19,20},
{1,0,15},
{1,0,16},
{15,0,16},
{0,15,17},
{0,16,18},
};
const int hand_joint_list[15][3] = {
{0,1,2},
{1,2,3},
{2,3,4},
{0,5,6},
{5,6,7},
{6,7,8},
{0,9,10},
{9,10,11},
{10,11,12},
{0,13,14},
{13,14,15},
{14,15,16},
{0,17,18},
{17,18,19},
{18,19,20},
};
const int body_link_list[22][2] = {
{1,0},
{2,1},
{3,2},
{4,3},
{5,1},
{6,5},
{7,6},
{8,1},
{9,8},
{10,9},
{11,10},
{12,8},
{13,12},
{14,13},
{15,0},
{16,0},
{17,15},
{18,16},
{19,21},
{20,19},
{22,11},
{23,22}
};
const int hand_link_list[20][2] = {
{1,0},
{2,1},
{3,2},
{4,3},
}

namespace openpose_ros {

    class OpenPoseROSIO
    {
        private:
            ros::NodeHandle nh_;
            ros::Publisher openpose_human_list_pub_;
            ros::Publisher openpose_human_list_pub_3D_;
			ros::Publisher marker_pub;
			ros::Publisher skeleton_pub;
            ros::Publisher motor_publisher;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;
            image_transport::Subscriber depth_sub_;
			ros::Subscriber info_sub;

            cv_bridge::CvImagePtr cv_img_ptr_;
            cv_bridge::CvImagePtr cv_depth_ptr_;
            std_msgs::Header image_header_;
            std_msgs::Header depth_header_;

            OpenPose* openpose_;

            bool display_output_flag_;
            bool print_keypoints_flag_;
            bool initial_keypoints_length_flag_;
            bool save_original_video_flag_;
            std::string original_video_file_name_;
            bool original_video_writer_initialized_;
            cv::VideoWriter original_video_writer_;

            bool save_openpose_video_flag_;
            std::string openpose_video_file_name_;
            bool openpose_video_writer_initialized_;
            cv::VideoWriter openpose_video_writer_;

			bool VIS_right, VIS_left, VIS_face, VIS_body, imgsync;

            int video_fps_;

			float fx, fy, cx, cy; // Camera Params

            std::vector<openpose_ros_msgs::OpenPoseHumanLink3D> last_links, links;
            std::vector<openpose_ros_msgs::OpenPoseHumanJoint3D> last_joints, joints;

        public:
            OpenPoseROSIO(OpenPose &openPose);

            ~OpenPoseROSIO(){}

            void processImage(const sensor_msgs::ImageConstPtr& msg);

            void convertImage(const sensor_msgs::ImageConstPtr& msg);

            void convertDepth(const sensor_msgs::ImageConstPtr& msg);

            void get_CamInfo(const sensor_msgs::CameraInfo& msg);

            std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> createDatum();

            bool display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            bool saveOriginalVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            bool saveOpenPoseVideo(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            cv_bridge::CvImagePtr& getCvImagePtr();

            void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            void publish(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

            void publish3D(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);
		
			void visualize(const std::vector<openpose_ros_msgs::OpenPoseHuman3D> humans);

			geometry_msgs::Point AddPoint(const openpose_ros_msgs::PointWithProb3D bodypart);

			bool PointISValid(const openpose_ros_msgs::PointWithProb3D& bodypart);

			openpose_ros_msgs::PointWithProb3D get3D(float, float, float);

            std::vector<openpose_ros_msgs::OpenPoseHuman3D> fixed_depth_filter(std::vector<openpose_ros_msgs::OpenPoseHuman3D> humans);

            std::vector<openpose_ros_msgs::OpenPoseHumanJoint3D> cal_joints(std::vector<openpose_ros_msgs::OpenPoseHuman3D> humans);

            std::vector<openpose_ros_msgs::OpenPoseHumanLink3D> cal_links(std::vector<openpose_ros_msgs::OpenPoseHuman3D> humans);

            std::vector<openpose_ros_msgs::OpenPoseHuman3D> error_correction_estimator(std::vector<openpose_ros_msgs::OpenPoseHuman3D> humans);

            void stop();
    };
}

#endif
