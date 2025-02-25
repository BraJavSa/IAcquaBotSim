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
        vel_sub = nh.subscribe("/cmd_vel", 10, &VelocityController::callbackVelDeseada, this);
        odom_sub = nh.subscribe("/boat/odom", 10, &VelocityController::callbackVelActual, this);
        signals_pub = nh.advertise<std_msgs::Float32MultiArray>("/wamv/signals", 10);
        
        ts = 0.02;
        k_d = 30.0;
        
        // Matrices constantes extraídas del archivo .mat
        M << 124.06341590941475772069679806008934974670410156250000L, 0, 0,
             0, 97.92262592424116007805423578247427940368652343750000L, 24.00184800122566031177484546788036823272705078125000L,
             0, 24.00184800122566031177484546788036823272705078125000L, 1024.58806252811655213008634746074676513671875000000000L;
        
        D << 209.70818898859158707637106999754905700683593750000000L, 0, 0,
             0, 24.99277753884350872226605133619159460067749023437500L, 1.37435567888283838300367278861813247203826904296875L,
             0, 1.37435567888283838300367278861813247203826904296875L, 795.58440621527017810876714065670967102050781250000000L;
        delta_3 = 24.00184800122566031177484546788036823272705078125000L;
        delta_5 = 62.97028138546249920182162895798683166503906250000000L;
        delta_6 = 33.94332221913634839438600465655326843261718750000000L;
        delta_7 = 29.02695916632567119108898623380810022354125976562500L;
    }
    
    void callbackVelDeseada(const geometry_msgs::Twist::ConstPtr& msg) {
        desired_velocity << msg->linear.x, msg->linear.y, msg->angular.z;
        acceleration_desired = (desired_velocity - prev_desired_velocity) / ts;
        prev_desired_velocity = desired_velocity;
        gammaFunction();
    }
    
    void callbackVelActual(const nav_msgs::Odometry::ConstPtr& msg) {
        actual_velocity << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z;
    }
    
    void gammaFunction() {
        Eigen::Matrix<long double, 3, 1> gamma = acceleration_desired + k_d * (desired_velocity - actual_velocity);
        torqueCalculation(gamma);
    }
    
    void torqueCalculation(const Eigen::Matrix<long double, 3, 1>& gamma) {
        Eigen::Matrix<long double, 3, 1> v = desired_velocity;
        Eigen::Matrix<long double, 3, 3> C;
        C << 0, -delta_5 * v(2), -delta_6 * v(1)- delta_3 * v(2),
             delta_5 * v(2), 0, delta_7 * v(0),
             delta_6 * v(1)+ delta_3 * v(2), -delta_7 * v(0), 0;
        
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
        // Asegúrate de que la conversión de tipo sea explícita si es necesario
        msg.data = {static_cast<float>(calcularCmd(T_left)), static_cast<float>(calcularCmd(T_right))};
        signals_pub.publish(msg);
    }
    
    long double calcularCmd(long double torque, long double tol = 1e-5) {
        // Capturamos 'this' para poder usar la función miembro 'calcularFuerzaPropulsor'
        auto error_func = [this, torque](long double cmd) {
            return calcularFuerzaPropulsor(cmd) - torque;
        };
    
        long double low = -1.0;
        long double high = 1.0;
        
        // Método de bisección
        while (high - low > tol) {
            long double mid = (low + high) / 2.0;
            if (error_func(mid) > 0) {
                high = mid;
            } else {
                low = mid;
            }
        }
    
        return (low + high) / 2.0;  // Valor de cmd con la máxima precisión
    }
    
    
    // Calcular la fuerza del propulsor
    long double calcularFuerzaPropulsor(long double cmd) {
        long double A_pos = 0.01L, K_pos = 59.82L, B_pos = 5.0L, v_pos = 0.38L, C_pos = 0.56L, M_pos = 0.28L;
        long double A_neg = -199.13L, K_neg = -0.09L, B_neg = 8.84L, v_neg = 5.34L, C_neg = 0.99L, M_neg = -0.57L;
        long double maxForceFwd = 200.0L, maxForceRev = -200.0L;
        long double T = 0;
        
        if (cmd > 0.01L) {
            // Caso positivo
            T = A_pos + (K_pos - A_pos) / std::pow((C_pos + std::exp(-B_pos * (cmd - M_pos))), (1.0L / v_pos));
        } else if (cmd < -0.01L) {
            // Caso negativo
            T = A_neg + (K_neg - A_neg) / std::pow((C_neg + std::exp(-B_neg * (cmd - M_neg))), (1.0L / v_neg));
        } else  {
            // Caso negativo
            T = 0;
        }
    
        // Controlar las fuerzas máximas
        if (T > maxForceFwd) {
            T = maxForceFwd;
        } else if (T < maxForceRev) {
            T = maxForceRev;
        }
    
        return T;
    }
    
    void run() {
        ros::spin();
    }
    
private:
    ros::Subscriber vel_sub, odom_sub;
    ros::Publisher signals_pub;
    long double ts, k_d;
    Eigen::Matrix<long double, 3, 1> desired_velocity, prev_desired_velocity, acceleration_desired, actual_velocity;
    Eigen::Matrix<long double, 3, 3> M, D;  // Usar long double para matrices
    long double delta_3, delta_5, delta_6, delta_7;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_listener");
    VelocityController controller;
    controller.run();
    return 0;
}
