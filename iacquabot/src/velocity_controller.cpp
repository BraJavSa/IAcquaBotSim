#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

class VelocityController {
public:
    VelocityController() {
        ros::NodeHandle nh;
        vel_sub = nh.subscribe("iacquabot/cmd_vel", 10, &VelocityController::callbackVelDeseada, this);
        odom_sub = nh.subscribe("/boat/odom", 10, &VelocityController::callbackVelActual, this);
        signals_pub = nh.advertise<std_msgs::Float32MultiArray>("/wamv/signals", 10);
        
        ts = 0.02L; // Periodo de muestreo 50Hz
        desired_velocity.setZero();
        prev_desired_velocity.setZero();
        acceleration_desired.setZero();
        actual_velocity.setZero();
        
        delta_1 = 131.406041018019408284089877270162L;
        delta_2 = 68.453244108018850511143682524562L;
        delta_3 = -0.536708819202315723373430955689L;
        delta_4 = 375.339840138962301807623589411378L;
        delta_5 = 68.570497467399988522629428189248L;
        delta_6 = 57.462687228812114881293382495642L;
        delta_7 = 11.107810238586326434528928075451L;
        delta_8 = 200.805889034405169013552949763834L;
        delta_9 = 265.054751383781422191532328724861L;
        delta_10 = 0.872524872563773867817360496701L;
        delta_11 = 255.400197330558313524306868202984L;
        
        M << delta_1, 0, 0,
             0, delta_2, delta_3,
             0, delta_3, delta_4;
        
        D << delta_8, 0, 0,
             0, delta_9, delta_10,
             0, delta_10, delta_11;
        
        k_d << 30.0L, 80.0L, 50.0L;
    }
    
    void callbackVelDeseada(const geometry_msgs::Twist::ConstPtr& msg) {
        desired_velocity << msg->linear.x, msg->linear.y, msg->angular.z;
    }
    
    void callbackVelActual(const nav_msgs::Odometry::ConstPtr& msg) {
        actual_velocity << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z;
    }
    
    void controlLoop() {
        acceleration_desired = (desired_velocity - prev_desired_velocity) / ts;
        prev_desired_velocity = desired_velocity;
        gammaFunction();
    }
    
    void gammaFunction() {
        Eigen::Matrix<long double, 3, 1> gamma = acceleration_desired + k_d.cwiseProduct(desired_velocity - actual_velocity);
        torqueCalculation(gamma);
    }
    
    void torqueCalculation(const Eigen::Matrix<long double, 3, 1>& gamma) {
        Eigen::Matrix<long double, 3, 1> v = desired_velocity;
        Eigen::Matrix<long double, 3, 3> C;
        C << 0, -delta_5 * v(2), -delta_6 * v(1) - delta_3 * v(2),
             delta_5 * v(2), 0, delta_7 * v(0),
             delta_6 * v(1) + delta_3 * v(2), -delta_7 * v(0), 0;
        
        Eigen::Matrix<long double, 3, 1> torque = M * gamma + C * v + D * v;
        double T_left, T_right;
        calculateMotorTorques(torque(0), torque(2), T_left, T_right);
        publishSignals(T_left, T_right);
    }
    
    void calculateMotorTorques(double T_u, double T_r, double& T_left, double& T_right, double d = 1.4) {
        T_left = (T_u / 4) - (T_r / (4 * d));
        T_right = (T_u / 4) + (T_r / (4 * d));
    }
    
    void publishSignals(double T_left, double T_right) {
        std_msgs::Float32MultiArray msg;
        msg.data = {static_cast<float>(T_left), static_cast<float>(T_right)};
        signals_pub.publish(msg);
    }
    
    void run() {
        ros::Rate rate(50); // 50 Hz
        while (ros::ok()) {
            ros::spinOnce();
            controlLoop();
            rate.sleep();
        }
    }
    
private:
    ros::Subscriber vel_sub, odom_sub;
    ros::Publisher signals_pub;
    long double ts;
    Eigen::Matrix<long double, 3, 1> k_d;
    Eigen::Matrix<long double, 3, 1> desired_velocity, prev_desired_velocity, acceleration_desired, actual_velocity;
    Eigen::Matrix<long double, 3, 3> M, D;
    long double delta_1, delta_2, delta_3, delta_4, delta_5, delta_6, delta_7, delta_8, delta_9, delta_10, delta_11;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_listener");
    VelocityController controller;
    controller.run();
    return 0;
}
