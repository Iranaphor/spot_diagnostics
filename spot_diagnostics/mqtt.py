#!/usr/bin/env python3
import os, socket, datetime
import json, msgpack, yaml

import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node

#from spot_msgs.msg import SpotStatus # <- find the correct source

# Define how to convert codes to human-readable values
MOTION = {0: 'skid-steering', 1: 'ackermann-steering', 2:'omni-directional-steering'}
VEHICLE = {0: 'active', 1: 'estop', 2: 'error?'}
CONTROL = {0: 'ROS', 1: 'Application', 2: 'Application Failed to Disconnect'}
VOLTAGE = {0: 'ERROR', 9: 'Critical (0.5 r)', 9.2: 'Very Low (0.5 y)', 9.5: 'Low (0.5 g)', 10.0: 'Medium Low (1.5 g)',  11.0: 'Medium (2.5)',  12.0: 'High (3.5)',  12.6: 'Full Charge'}

class MqttPublisher(Node):

    def __init__(self):
        super().__init__('mqtt_diagnostics')

        # Define all the details for the MQTT broker
        self.mqtt_ip = os.getenv('MQTT_BROKER_IP', '')
        self.mqtt_port = int(os.getenv('MQTT_BROKER_PORT', 8883))
        self.mqtt_encoding = os.getenv('MQTT_ENCODING', 'json')
        self.mqtt_client = None

        # Specify the loading and dumping functions
        self.dumps = msgpack.dumps if self.mqtt_encoding == 'msgpack' else json.dumps
        self.loads = msgpack.loads if self.mqtt_encoding == 'msgpack' else json.loads

        # Define source information
        self.robot_name = os.getenv('ROBOT_NAME', socket.gethostname())

        # Initiate connections to ROS and MQTT
        self.connect_to_mqtt()
        self.connect_to_ros()

    def connect_to_mqtt(self):
        # MQTT management functions
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, self.robot_name+'_status_logger')
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port)
        self.mqtt_client.loop_start()


    def on_connect(self, client, userdata, flags, rc):
        print(" MQTT ->     | Connected")

    def on_message(self, client, userdata, msg):
        print(" MQTT        | Message received ["+msg.topic+"]")

    def connect_to_ros(self):
        self.sub = self.create_subscription(SpotStatus, '/spot_status', self.status_cb, 10)

    def status_cb(self, msg):
        ns = 'agilex/%s/status/'%self.robot_name
        self.mqtt_client.publish(ns+'battery_voltage', msg.battery_voltage, retain=True)
        
        v = [val for volt, val in VOLTAGE.items() if msg.battery_voltage >= volt][-1]
        self.mqtt_client.publish(ns+'battery_inidcator', v, retain=True)

        self.mqtt_client.publish(ns+'motion_mode', MOTION[msg.motion_mode])
        self.mqtt_client.publish(ns+'vehicle_state', VEHICLE[msg.vehicle_state])
        self.mqtt_client.publish(ns+'control_mode', CONTROL[msg.control_mode])
        self.mqtt_client.publish(ns+'error_code', msg.error_code, retain=True)

        t = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
        self.mqtt_client.publish(ns+'last_online', str(t), retain=True)


def main(args=None):
    rclpy.init(args=args)

    MP = MqttPublisher()
    rclpy.spin(MP)

    MP.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
