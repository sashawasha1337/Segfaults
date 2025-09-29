'''
Helper class to manage polling and pushing status updates (battery voltage, WiFi strength) over a WebRTC data channel.
'''

import json, re, subprocess

class StatusPusher:
    def __init__(self, node, status_channel_getter):
        self.node = node
        self.status_channel_getter = status_channel_getter

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
        peer_session = self.status_channel_getter()
        sc = getattr(peer_session, 'status_channel', None)
        self.node.get_logger().info(f"Status channel object id: {id(sc) if sc else 'None'}")

        if sc and sc.readyState == "open":
            self.node.get_logger().info(f'Sending status on channel {sc.label}, state={sc.readyState}')
            payload_json = json.dumps(payload)
            sc.send(payload_json)
            self.node.get_logger().info(f'Status payload: {payload_json}')
            self.node.get_logger().info(f'Status buffer amount: {sc.bufferedAmount}')
            self.node.get_logger().info('Status sent over data channel')
        else:
            self.node.get_logger().warning('Data channel not open, cannot send status')
