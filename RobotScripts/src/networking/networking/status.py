'''
Helper class to manage polling and pushing status updates (battery voltage, WiFi strength) over a WebRTC data channel.
'''

import json, re, subprocess

class StatusPusher:
    def __init__(self, node, data_channel_getter):
        self.node = node
        self.data_channel_getter = data_channel_getter

    def battery_callback(self, msg):
        self.node.battery_voltage = msg.voltage
        self.node.get_logger().info(f'Battery voltage: {self.node.battery_voltage} V')

    def get_wifi_strength(self):
        try:
            result = subprocess.run(['iwconfig'], capture_output=True, text=True)
            match = re.search(r'Signal level[=:](-?\d+)\s*dBm', result.stdout)
            return int(match.group(1)) if match else None
        except Exception as e:
            self.node.get_logger().error(f'Error getting WiFi strength: {str(e)}')
            return None

    def push_status(self):
        wifi_strength = self.get_wifi_strength()

        self.node.get_logger().info(f'Wifi Strength: {wifi_strength}')

        payload = {
            'type': 'status',
            'batteryVoltage': self.node.battery_voltage,
            'wifiStrength': wifi_strength,
        }
        dc = self.data_channel_getter()
        if dc and dc.data_channel and dc.data_channel.readyState == "open":
            dc.data_channel.send(json.dumps(payload))
            self.node.get_logger().info('Status sent over data channel')
