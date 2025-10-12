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
        self.node.get_logger().debug(f'Battery voltage: {self.node.battery_voltage} V', throttle_duration_sec=10)

    def get_wifi_strength(self):
        try:
            result = subprocess.run(['iwconfig'], capture_output=True, text=True)
            match = re.search(r'Signal level[=:](-?\d+)\s*dBm', result.stdout)
            return int(match.group(1)) if match else None
        except Exception as e:
            self.node.get_logger().error(f'Error getting WiFi strength: {str(e)}')
            return None

    def push_status(self):
        peer_session = self.data_channel_getter()
        if not peer_session:
            self.node.get_logger().error('No active peer session, cannot send status')
            return


        wifi_strength = self.get_wifi_strength()

        self.node.get_logger().debug(f'Wifi Strength: {wifi_strength}', throttle_duration_sec=10)

        payload = {
            'type': 'status',
            'batteryVoltage': self.node.battery_voltage,
            'wifiStrength': wifi_strength,
        }

        dc = getattr(peer_session, 'data_channel', None)
        self.node.get_logger().info(f"Robot channel object id: {id(dc) if dc else 'None'}")
        
        if dc and dc.readyState == "open":
            payload_json = json.dumps(payload)
            dc.send(payload_json)
            self.node.get_logger().info(f'Status sent: {payload_json}')
        else:
            self.node.get_logger().error('Data channel not open, cannot send status')