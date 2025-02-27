#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def callback(data):
    # Creamos un objeto de servicio de SetModelState
    try:
        # Conectamos al servicio de SetModelState
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Preparamos el estado del modelo para enviar
        model_state = ModelState()
        rospy.loginfo("posicion recibida")
        model_state.model_name = 'red_totem'

        model_state.pose = data  # Pose recibida del tópico
        model_state.reference_frame = "world"  # Añadir esto


        # Usamos el servicio para actualizar la posición del modelo
        set_model_state(model_state)
    except rospy.ServiceException as e:
        rospy.logerr("Error al actualizar la posición del modelo: %s", e)

def listener():
    # Inicializamos el nodo de ROS
    rospy.init_node('update_red_totem_position', anonymous=True)

    # Nos suscribimos al tópico '/iacquabot/desired_position' para recibir las posiciones
    rospy.Subscriber('/iacquabot/desired_position', Pose, callback)

    # Mantenemos el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    listener()
