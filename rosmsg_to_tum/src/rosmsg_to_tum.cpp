#include <ros/ros.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;

string viwo_path = "/home/jaeho/SLAM_project/viwo.txt";
string orb_path = "/home/jaeho/SLAM_project/orb.txt";
string gt_path = "/home/jaeho/SLAM_project/gt.txt";
string wheel_path = "/home/jaeho/SLAM_project/wheel.txt";
string wheel_imu_path = "/home/jaeho/SLAM_project/wheel_imu.txt";
string vio_path = "/home/jaeho/SLAM_project/vio.txt";
string test_path = "/home/jaeho/SLAM_project/my.txt";
string viwo_range_path = "/home/jaeho/SLAM_project/viwo_range.txt";
string viwlo_path = "/home/jaeho/SLAM_project/viwlo.txt";

void my_callback(const nav_msgs::OdometryConstPtr & msg){
    std::stringstream ss;
    string timestamp = to_string(msg->header.stamp.sec) + "." + to_string(msg->header.stamp.nsec) + " ";
    string x = to_string(msg->pose.pose.position.x) + " ";
    string y = to_string(msg->pose.pose.position.y) + " ";
    string z= to_string(msg->pose.pose.position.z) + " ";
    string q_x = to_string(msg->pose.pose.orientation.x) + " ";
    string q_y = to_string(msg->pose.pose.orientation.y) + " ";
    string q_z = to_string(msg->pose.pose.orientation.z) + " ";
    string q_w = to_string(msg->pose.pose.orientation.w) + "\n";
    ofstream writeMy;
    writeMy.open(test_path,ios::app);

    if(writeMy.is_open()){
        writeMy << timestamp + x + y + z + q_x + q_y + q_z + q_w;
        writeMy.close();
    }
}

void viwo_range_callback(const nav_msgs::OdometryConstPtr & msg){
    std::stringstream ss;
    string timestamp = to_string(msg->header.stamp.sec) + "." + to_string(msg->header.stamp.nsec) + " ";
    string x = to_string(msg->pose.pose.position.x) + " ";
    string y = to_string(msg->pose.pose.position.y) + " ";
    string z= to_string(msg->pose.pose.position.z) + " ";
    string q_x = to_string(msg->pose.pose.orientation.x) + " ";
    string q_y = to_string(msg->pose.pose.orientation.y) + " ";
    string q_z = to_string(msg->pose.pose.orientation.z) + " ";
    string q_w = to_string(msg->pose.pose.orientation.w) + "\n";
    ofstream writeMy;
    writeMy.open(viwo_range_path,ios::app);

    if(writeMy.is_open()){
        writeMy << timestamp + x + y + z + q_x + q_y + q_z + q_w;
        writeMy.close();
    }
}
void viwlo_callback(const nav_msgs::OdometryConstPtr & msg){
    std::stringstream ss;
    string timestamp = to_string(msg->header.stamp.sec) + "." + to_string(msg->header.stamp.nsec) + " ";
    string x = to_string(msg->pose.pose.position.x) + " ";
    string y = to_string(msg->pose.pose.position.y) + " ";
    string z= to_string(msg->pose.pose.position.z) + " ";
    string q_x = to_string(msg->pose.pose.orientation.x) + " ";
    string q_y = to_string(msg->pose.pose.orientation.y) + " ";
    string q_z = to_string(msg->pose.pose.orientation.z) + " ";
    string q_w = to_string(msg->pose.pose.orientation.w) + "\n";
    ofstream writeMy;
    writeMy.open(viwlo_path,ios::app);

    if(writeMy.is_open()){
        writeMy << timestamp + x + y + z + q_x + q_y + q_z + q_w;
        writeMy.close();
    }
}

void viwo_callback(const nav_msgs::OdometryConstPtr & msg){
    std::stringstream ss;
    string timestamp = to_string(msg->header.stamp.sec) + "." + to_string(msg->header.stamp.nsec) + " ";
    string x = to_string(msg->pose.pose.position.x) + " ";
    string y = to_string(msg->pose.pose.position.y) + " ";
    string z= to_string(msg->pose.pose.position.z) + " ";
    string q_x = to_string(msg->pose.pose.orientation.x) + " ";
    string q_y = to_string(msg->pose.pose.orientation.y) + " ";
    string q_z = to_string(msg->pose.pose.orientation.z) + " ";
    string q_w = to_string(msg->pose.pose.orientation.w) + "\n";
    ofstream writeEKF;
    writeEKF.open(viwo_path,ios::app);

    if(writeEKF.is_open()){
        writeEKF << timestamp + x + y + z + q_x + q_y + q_z + q_w;
        writeEKF.close();
    }
}
void vio_callback(const nav_msgs::OdometryConstPtr & msg){
    std::stringstream ss;
    string timestamp = to_string(msg->header.stamp.sec) + "." + to_string(msg->header.stamp.nsec) + " ";
    string x = to_string(msg->pose.pose.position.x) + " ";
    string y = to_string(msg->pose.pose.position.y) + " ";
    string z= to_string(msg->pose.pose.position.z) + " ";
    string q_x = to_string(msg->pose.pose.orientation.x) + " ";
    string q_y = to_string(msg->pose.pose.orientation.y) + " ";
    string q_z = to_string(msg->pose.pose.orientation.z) + " ";
    string q_w = to_string(msg->pose.pose.orientation.w) + "\n";
    ofstream writeVIO;
    writeVIO.open(vio_path,ios::app);

    if(writeVIO.is_open()){
        writeVIO << timestamp + x + y + z + q_x + q_y + q_z + q_w;
        writeVIO.close();
    }
}
void orb_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg){
    std::stringstream ss;
    string timestamp = to_string(msg->header.stamp.sec) + "." + to_string(msg->header.stamp.nsec) + " ";
    string x = to_string(msg->pose.pose.position.x) + " ";
    string y = to_string(msg->pose.pose.position.y) + " ";
    string z= to_string(msg->pose.pose.position.z) + " ";
    string q_x = to_string(msg->pose.pose.orientation.x) + " ";
    string q_y = to_string(msg->pose.pose.orientation.y) + " ";
    string q_z = to_string(msg->pose.pose.orientation.z) + " ";
    string q_w = to_string(msg->pose.pose.orientation.w) + "\n";
    ofstream writeORB;
    writeORB.open(orb_path,ios::app);

    if(writeORB.is_open()){
        writeORB << timestamp + x + y + z + q_x + q_y + q_z + q_w;
        writeORB.close();
    }

}
void wheel_callback(const nav_msgs::OdometryConstPtr & msg){
    std::stringstream ss;
    string timestamp = to_string(msg->header.stamp.sec) + "." + to_string(msg->header.stamp.nsec) + " ";
    string x = to_string(msg->pose.pose.position.x) + " ";
    string y = to_string(msg->pose.pose.position.y) + " ";
    string z= to_string(msg->pose.pose.position.z) + " ";
    string q_x = to_string(msg->pose.pose.orientation.x) + " ";
    string q_y = to_string(msg->pose.pose.orientation.y) + " ";
    string q_z = to_string(msg->pose.pose.orientation.z) + " ";
    string q_w = to_string(msg->pose.pose.orientation.w) + "\n";
    ofstream writeWheel;
    writeWheel.open(wheel_path,ios::app);

    if(writeWheel.is_open()){
        writeWheel << timestamp + x + y + z + q_x + q_y + q_z + q_w;
        writeWheel.close();
    }

}

void gt_callback(const nav_msgs::OdometryConstPtr & msg){
    std::stringstream ss;
    string timestamp = to_string(msg->header.stamp.sec) + "." + to_string(msg->header.stamp.nsec) + " ";
    string x = to_string(msg->pose.pose.position.x) + " ";
    string y = to_string(msg->pose.pose.position.y) + " ";
    string z= to_string(msg->pose.pose.position.z) + " ";
    string q_x = to_string(msg->pose.pose.orientation.x) + " ";
    string q_y = to_string(msg->pose.pose.orientation.y) + " ";
    string q_z = to_string(msg->pose.pose.orientation.z) + " ";
    string q_w = to_string(msg->pose.pose.orientation.w) + "\n";
    ofstream writeGT;
    writeGT.open(gt_path,ios::app);

    if(writeGT.is_open()){
        writeGT << timestamp + x + y + z + q_x + q_y + q_z + q_w;
        writeGT.close();
    }

}

void wheel_vo_callback(const nav_msgs::OdometryConstPtr & msg){
    std::stringstream ss;
    string timestamp = to_string(msg->header.stamp.sec) + "." + to_string(msg->header.stamp.nsec) + " ";
    string x = to_string(msg->pose.pose.position.x) + " ";
    string y = to_string(msg->pose.pose.position.y) + " ";
    string z= to_string(msg->pose.pose.position.z) + " ";
    string q_x = to_string(msg->pose.pose.orientation.x) + " ";
    string q_y = to_string(msg->pose.pose.orientation.y) + " ";
    string q_z = to_string(msg->pose.pose.orientation.z) + " ";
    string q_w = to_string(msg->pose.pose.orientation.w) + "\n";
    ofstream write_wheel_imu;
    write_wheel_imu.open(wheel_imu_path,ios::app);

    if(write_wheel_imu.is_open()){
        write_wheel_imu << timestamp + x + y + z + q_x + q_y + q_z + q_w;
        write_wheel_imu.close();
    }
}

int main(int argc, char ** argv){
    ros::init(argc,argv,"rosmsg_to_tum");
    ros::NodeHandle nh;
    ros::Subscriber viwo_range_sub = nh.subscribe<nav_msgs::Odometry>("/viwo_range",1,viwo_range_callback); // /odometry/filtered
    ros::Subscriber viwo_sub = nh.subscribe<nav_msgs::Odometry>("/viwo",1,viwo_callback); // /odometry/filtered
    ros::Subscriber viwo_decay_sub = nh.subscribe<nav_msgs::Odometry>("/viwlo",1,viwlo_callback); // /odometry/filtered
    ros::Subscriber my_sub = nh.subscribe<nav_msgs::Odometry>("/filtered_pose",1,my_callback); // /odometry/filtered
    ros::Subscriber gt_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth",1,gt_callback);
    ros::Subscriber wheel_sub = nh.subscribe<nav_msgs::Odometry>("/odom/unfiltered",1,wheel_callback);
    ros::Subscriber orb_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/orb_slam2_stereo/pose",1,orb_callback);
    ros::Subscriber wheel_imu_sub = nh.subscribe<nav_msgs::Odometry>("/wheel_imu",1,wheel_vo_callback);
    ros::Subscriber vio_sub = nh.subscribe<nav_msgs::Odometry>("/vio",1, vio_callback);
    ros::spin();
    
    return 0;

}
