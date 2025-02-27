#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped  # Para recibir la posición deseada

class TrajectoryAnimation:
    def __init__(self, fr=50, tf=360):
        self.fr = fr
        self.ts = 1 / fr
        self.tf = tf
        self.t = np.arange(0, tf + self.ts, self.ts)
        
        # Definir las trayectorias Xd y Yd
        self.Xd = 80 * np.sin(0.04 * self.t)
        self.Yd = 40 * np.sin(0.02 * self.t)

        # Configurar la figura y el eje
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-90, 90)
        self.ax.set_ylim(-50, 50)

        # Dibujar la trayectoria completa en azul
        self.ax.plot(self.Xd, self.Yd, 'b-', lw=2)

        # Crear el punto verde para la posición del barco
        self.boat_point, = self.ax.plot([], [], 'ro', markersize=10)

        # Crear el punto amarillo para la posición deseada
        self.desired_point, = self.ax.plot([], [], 'go', markersize=10)

        # Inicializar ROS y la suscripción a los temas de odometría y posición deseada
        rospy.init_node('trajectory_animation')
        rospy.Subscriber('/boat/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/desired_position', PoseStamped, self.desired_position_callback)

        # Usar rospy.Rate para controlar la frecuencia
        self.rate = rospy.Rate(self.fr)

        # Inicializar las posiciones actuales del barco y la posición deseada
        self.boat_x = None
        self.boat_y = None
        self.desired_x = None
        self.desired_y = None

    def odom_callback(self, msg):
        # Extraer las posiciones x y y del mensaje de odometría
        self.boat_x = msg.pose.pose.position.x
        self.boat_y = msg.pose.pose.position.y

    def desired_position_callback(self, msg):
        # Extraer las posiciones x y y del mensaje de la posición deseada
        self.desired_x = msg.pose.position.x
        self.desired_y = msg.pose.position.y

    def init_func(self):
        self.boat_point.set_data([], [])
        self.desired_point.set_data([], [])
        return self.boat_point, self.desired_point

    def update(self, frame):
        # Actualizar la posición del barco (punto verde)
        if self.boat_x is not None and self.boat_y is not None:
            self.boat_point.set_data(self.boat_x, self.boat_y)

        # Actualizar la posición deseada (punto amarillo)
        if self.desired_x is not None and self.desired_y is not None:
            self.desired_point.set_data(self.desired_x, self.desired_y)

        return self.boat_point, self.desired_point

    def animate(self):
        # Función para la animación
        anim = FuncAnimation(self.fig, self.update, frames=len(self.t), init_func=self.init_func, blit=True, interval=self.ts)

        while not rospy.is_shutdown():
            anim.event_source.start()
            plt.pause(self.ts)  # Controla la frecuencia de actualización
            self.rate.sleep()  # Mantener la tasa de actualización a 50Hz
        plt.show()

# Crear la animación
animation = TrajectoryAnimation()
animation.animate()
