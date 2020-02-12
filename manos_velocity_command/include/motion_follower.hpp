#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <velocity_extraction_msg/TimeArray.h>
#include <std_msgs/Time.h>
#include <trajectory_smoothing_msg/SmoothRWristCoordsWithRespectToBase.h>
#include <velocity_extraction_msg/PointsStampedArray.h>
#include <stdlib.h>
#include <memory>
#include <vector>
#include <numeric>
#include <cmath>
#include <iomanip>

static ros::Publisher pub, vis_human, vis_robot, pub_robot_state, pub_points_human, pub_points_robot;
static ros::Time beginTime;
static ros::Time currentTime;

// std::shared_ptr<cartesian_state_msgs::PoseTwist> robot_state = boost::make_shared<cartesian_state_msgs::PoseTwist>();
static std_msgs::TimePtr th = boost::make_shared<std_msgs::Time>();
static std_msgs::TimePtr tr = boost::make_shared<std_msgs::Time>();

static velocity_extraction_msg::PointsStampedArray human_points;
static velocity_extraction_msg::PointsStampedArray robot_points;

static geometry_msgs::PointPtr temp_point = boost::make_shared<geometry_msgs::Point>();
static geometry_msgs::PointStampedPtr temp_point_human = boost::make_shared<geometry_msgs::PointStamped>();
static geometry_msgs::PointStampedPtr temp_point_robot = boost::make_shared<geometry_msgs::PointStamped>();

static geometry_msgs::PointStampedPtr desired_robot_position = boost::make_shared<geometry_msgs::PointStamped>();
static geometry_msgs::PointStampedPtr robot_state = boost::make_shared<geometry_msgs::PointStamped>();

static geometry_msgs::TwistPtr vel_control = boost::make_shared<geometry_msgs::Twist>();
static geometry_msgs::TwistPtr safe_vel_control = boost::make_shared<geometry_msgs::Twist>();

static visualization_msgs::MarkerPtr marker_human = boost::make_shared<visualization_msgs::Marker>();
static visualization_msgs::MarkerPtr marker_robot = boost::make_shared<visualization_msgs::Marker>();

static velocity_extraction_msg::TimeArray human_time, robot_time;

static std::vector<ros::Duration> diff_time;

static std::shared_ptr<std::vector<double>> v1 = std::make_shared<std::vector<double>>();
static std::shared_ptr<std::vector<double>> v2 = std::make_shared<std::vector<double>>();

static float D=3, init_x, init_y, init_z, sleep_rate, init_gain;
static bool init_flag = true, human_flag = false, time_pub = true, openpose_second=false;
static ros::Time human_point_time;
static double control_time;


double euclidean_distance (std::shared_ptr<std::vector<double>> v1, std::shared_ptr<std::vector<double>> v2);