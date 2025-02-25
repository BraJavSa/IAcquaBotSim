#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>  // Incluimos este encabezado para Pose
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

class PositionController {
public:
    PositionController() {
        ros::NodeHandle nh;
        pos_sub = nh.subscribe("/desired_position", 10, &PositionController::callbackPosDeseada, this);
        odom_sub = nh.subscribe("/boat/odom", 10, &PositionController::callbackPosActual, this);
        vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        ts = 0.02; // Intervalo de tiempo en segundos
        k_p << 10.0, 10.0, 10.0;  // Ganancia proporcional para cada eje

        prev_pos_deseada.setZero(); // Inicializamos la posición deseada pasada en cero
    }

    void callbackPosDeseada(const geometry_msgs::Pose::ConstPtr& msg) {
        // Extraemos la posición deseada desde el mensaje de tipo Pose
        desired_position << msg->position.x, msg->position.y, 0;  // Suponemos que no hay componente Z

        // Calculamos la velocidad deseada utilizando la fórmula indicada
        desired_velocity = (desired_position - prev_pos_deseada) / ts + k_p.cwiseProduct(actual_position - desired_position);
        
        prev_pos_deseada = desired_position;  // Actualizamos la posición deseada pasada
        publishVelocity();
    }

    void callbackPosActual(const nav_msgs::Odometry::ConstPtr& msg) {
        // Obtenemos la posición actual del vehículo desde el Odometry
        actual_position << msg->pose.pose.position.x, msg->pose.pose.position.y, 0;  // Suponemos que no tenemos Z en la odometría
    }

    void publishVelocity() {
        geometry_msgs::Twist msg;
        msg.linear.x = desired_velocity(0);
        msg.linear.y = desired_velocity(1);
        msg.angular.z = desired_velocity(2);

        vel_pub.publish(msg);
    }

    void run() {
        ros::spin();
    }

private:
    ros::Subscriber pos_sub, odom_sub;
    ros::Publisher vel_pub;
    long double ts;
    Eigen::Matrix<long double, 3, 1> k_p;  // Vector de ganancias proporcionales
    Eigen::Matrix<long double, 3, 1> desired_position, prev_pos_deseada;
    Eigen::Matrix<long double, 3, 1> actual_position, desired_velocity;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "position_controller");
    PositionController controller;
    controller.run();
    return 0;
}
