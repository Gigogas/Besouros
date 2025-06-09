#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Temperature
from functools import partial

# Lista de tópicos potenciais para temperatura.
POTENTIAL_TEMP_TOPICS = [
    "/mavros/imu/temperature_imu",      # Temp do primeiro IMU (geralmente o aquecido)
    "/mavros/imu/temperature_imu2",     # Temp do segundo IMU
    "/mavros/imu/temperature_imu3",     # Temp do terceiro IMU (se houver)
    "/mavros/imu/temperature_baro",     # Temp do barômetro
    "/mavros/imu/temperature_baro2",    # Temp do segundo barômetro
]

def temp_callback(data, topic_name):
    """Callback para exibir dados de temperatura de um tópico específico."""
    rospy.loginfo_throttle(5, "--- Temperatura de {} ---".format(topic_name))
    rospy.loginfo_throttle(5, "  Temperatura: {:.2f} C".format(data.temperature))
    rospy.loginfo_throttle(5, "  Frame ID: {}".format(data.header.frame_id))
    rospy.loginfo_throttle(5, "-----------------------------------")


def temperature_listener():
    rospy.init_node('multi_temperature_listener_node', anonymous=True)
    rospy.loginfo("Procurando por topicos de temperatura disponiveis...")

    available_topics = rospy.get_published_topics()
    topic_types = {name: msg_type for name, msg_type in available_topics}
    found_any_topic = False

    for topic in POTENTIAL_TEMP_TOPICS:
        if topic in topic_types and topic_types[topic] == 'sensor_msgs/Temperature':
            rospy.loginfo("Encontrado! Assinando o topico: {}".format(topic))
            rospy.Subscriber(topic, Temperature, partial(temp_callback, topic_name=topic))
            found_any_topic = True

    if not found_any_topic:
        rospy.logwarn("Nenhum topico de temperatura da lista foi encontrado.")
        rospy.logwarn("Verifique se o MAVROS esta rodando e se os streams de dados estao ativos.")
    else:
        rospy.loginfo("Escutando dados de todos os sensores de temperatura encontrados.")
        rospy.spin()

if __name__ == '__main__':
    try:
        temperature_listener()
    except rospy.ROSInterruptException:
        pass