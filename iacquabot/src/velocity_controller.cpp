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
            
            // Matrices constantes extraídas del archivo .mat
            M << delta_1, 0, 0,
                 0, delta_2, delta_3,
                 0, delta_3, delta_4;
            
            D << delta_8, 0, 0,
                 0, delta_9, delta_10,
                 0, delta_10, delta_11;
            
            
    
            // Define diferentes valores de k_d para cada componente
            k_d << 30.0L, 80.0L, 50.0L;  // Por ejemplo, puedes asignar diferentes valores para cada fila
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
            // Usamos k_d para cada componente de la velocidad en el cálculo de gamma
            Eigen::Matrix<long double, 3, 1> gamma = acceleration_desired + k_d.cwiseProduct(desired_velocity - actual_velocity);
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
            msg.data = {static_cast<float>(calcularCmd(T_left)), static_cast<float>(calcularCmd(T_right))};
            signals_pub.publish(msg);
        }
        
        long double calcularCmd(long double torque, long double tol = 1e-5) {
            auto error_func = [this, torque](long double cmd) {
                return calcularFuerzaPropulsor(cmd) - torque;
            };
        
            long double low = -1.0;
            long double high = 1.0;
            
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
        
        long double calcularFuerzaPropulsor(long double cmd) {
            long double A_pos = 0.01L, K_pos = 59.82L, B_pos = 5.0L, v_pos = 0.38L, C_pos = 0.56L, M_pos = 0.28L;
            long double A_neg = -199.13L, K_neg = -0.09L, B_neg = 8.84L, v_neg = 5.34L, C_neg = 0.99L, M_neg = -0.57L;
            long double maxForceFwd = 200.0L, maxForceRev = -200.0L;
            long double T = 0;
            
            if (cmd > 0.01L) {
                T = A_pos + (K_pos - A_pos) / std::pow((C_pos + std::exp(-B_pos * (cmd - M_pos))), (1.0L / v_pos));
            } else if (cmd < -0.01L) {
                T = A_neg + (K_neg - A_neg) / std::pow((C_neg + std::exp(-B_neg * (cmd - M_neg))), (1.0L / v_neg));
            } else  {
                T = 0;
            }
        
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
        long double ts;
        Eigen::Matrix<long double, 3, 1> k_d;  // Vector k_d de 3 componentes
        Eigen::Matrix<long double, 3, 1> desired_velocity, prev_desired_velocity, acceleration_desired, actual_velocity;
        Eigen::Matrix<long double, 3, 3> M, D;
        long double delta_1, delta_2, delta_3,delta_4, delta_5, delta_6, delta_7, delta_8, delta_9, delta_10, delta_11;
    };

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_listener");
    VelocityController controller;
    controller.run();
    return 0;
}
