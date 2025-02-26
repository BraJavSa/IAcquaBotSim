import rospy
from geometry_msgs.msg import Pose
import math

# Parámetros de la trayectoria
width = 20.0  # Ancho del 8 en metros
height = 40.0  # Alto del 8 en metros
total_time = 80.0  # Tiempo total en segundos
frequency = 50  # Frecuencia de publicación (50 Hz)

# Creamos un nodo ROS
rospy.init_node('eight_trajectory_publisher')

# Publicador
pub = rospy.Publisher('/iacquabot/desired_position', Pose, queue_size=10)

# Calculamos el número de iteraciones (frecuencia * tiempo total)
iterations = int(total_time * frequency)

# Variables para el control de la trayectoria
center_y = height / 2
center_x = width / 2

def generate_eight_trajectory(t):
    # Parámetros de la trayectoria en forma de 8
    # Senoide en forma de 8, dividida en dos partes para crear la forma
    x = width * math.sin(math.pi * t / total_time)
    y = center_y * math.sin(2 * math.pi * t / total_time)

    return x, y

def main():
    rate = rospy.Rate(frequency)

    for i in range(iterations):
        # Calcular el tiempo normalizado (t / total_time)
        t = i * total_time / iterations

        # Generar la posición en la trayectoria
        x, y = generate_eight_trajectory(t)

        # Crear el mensaje Pose
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.8  # Asumimos que no hay movimiento en Z

        # Orientación (si no se necesita, puede quedarse en 0)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        # Publicar el mensaje
        pub.publish(pose)

        # Esperar para mantener la frecuencia deseada
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
