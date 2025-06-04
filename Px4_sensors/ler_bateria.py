#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import BatteryState

def battery_callback(data):
    rospy.loginfo_throttle(10, "----- Status da Bateria -----") # Loga a cada 10 segundos
    rospy.loginfo_throttle(10, "Tensao (V): {:.2f}".format(data.voltage))
    rospy.loginfo_throttle(10, "Corrente (A): {:.2f}".format(data.current)) # Pode ser negativo se carregando
    rospy.loginfo_throttle(10, "Carga Restante (Ah): {:.2f}".format(data.charge)) # Capacidade restante
    rospy.loginfo_throttle(10, "Capacidade Total (Ah): {:.2f}".format(data.capacity)) # Capacidade de design
    rospy.loginfo_throttle(10, "Porcentagem Restante: {:.1f}%".format(data.percentage * 100))
    rospy.loginfo_throttle(10, "Status de Energia: {}".format(data.power_supply_status)) # ex: POWER_SUPPLY_STATUS_DISCHARGING
    rospy.loginfo_throttle(10, "Saude da Bateria: {}".format(data.power_supply_health))
    rospy.loginfo_throttle(10, "Tecnologia da Bateria: {}".format(data.power_supply_technology))
    rospy.loginfo_throttle(10, "Presente: {}".format(data.present))
    if data.cell_voltage: #lista com as tensões de cada célula, se disponível
        cell_voltages_str = ", ".join(["{:.2f}V".format(v) for v in data.cell_voltage])
        rospy.loginfo_throttle(10, "Tensao das Celulas: [{}]".format(cell_voltages_str))
    rospy.loginfo_throttle(10, "---------------------------")

def battery_listener():
    rospy.init_node('battery_listener_node', anonymous=True)
    rospy.Subscriber('/mavros/battery', BatteryState, battery_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        battery_listener()
    except rospy.ROSInterruptException:
        pass