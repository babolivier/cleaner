#include "ros/ros.h"
#include "rosgraph_msgs/Log.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
ros::Subscriber log_subscriber;
turtlesim::Pose turtlesim_pose;

double PI = 3.1415926535898;
bool didJustHit = false;

using namespace std;

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double relative_angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
double setDesiredOrientation(double desired_angle_radians);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void logCallback(const rosgraph_msgs::Log & log_message);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
double getDistance(double x1, double y1, double x2, double y2);
void wander();

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_cleaner");
    ros::NodeHandle n;

    double speed;
    double distance;
    bool isForward;

    double angle;
    bool dir;

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
    log_subscriber = n.subscribe("/rosout", 10, logCallback);

    while(ros::ok()) {
        if (didJustHit) {
            move(3, 1, false);
            angle = rand() % 360;
            dir = rand() % 2 == 1;
            rotate(degrees2radians(90), degrees2radians(angle), dir);
            didJustHit = false;
        } else {
            move(3, 0.1, true);
        }
    }

    ros::spin();

    return 0;
}

void logCallback(const rosgraph_msgs::Log & log_message) {
    if (log_message.level == log_message.WARN && log_message.name == "/turtlesim") {
        didJustHit = true;
    }
}

void move(double speed, double distance, bool isForward) {
    geometry_msgs::Twist vel_msg;

    if(isForward) {
        vel_msg.linear.x = abs(speed);
    } else {
        vel_msg.linear.x = -abs(speed);
    }

    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    double t0 = ros::Time::now().toSec();
    double current_distance = 0;
    ros::Rate loop_rate(100);
    do {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while(current_distance < distance);

    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}

void rotate(double angular_speed, double relative_angle, bool clockwise) {
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    if(clockwise) {
        vel_msg.angular.z = -abs(angular_speed);
    } else {
        vel_msg.angular.z = abs(angular_speed);
    }

    double t0 = ros::Time::now().toSec();
    double current_angle = 0;
    ros::Rate loop_rate(100);
    do {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while(current_angle < relative_angle);

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees) {
    return angle_in_degrees * PI / 180.0;
}

double setDesiredOrientation(double desired_angle_radians) {
    double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
    bool clockwise = ((relative_angle_radians<0)?true:false);
    cout<<desired_angle_radians <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
    rotate (abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message) {
    turtlesim_pose.x=pose_message->x;
    turtlesim_pose.y=pose_message->y;
    turtlesim_pose.theta=pose_message->theta;
}

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){
    geometry_msgs::Twist vel_msg;

    ros::Rate loop_rate(10);
    do {

        vel_msg.linear.x = 1.5*getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 4*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta);

        velocity_publisher.publish(vel_msg);

        ros::spinOnce();
        loop_rate.sleep();

    } while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance);
    cout<<"end move goal"<<endl;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

double getDistance(double x1, double y1, double x2, double y2){
    return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}
