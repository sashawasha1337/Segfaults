# sudo apt install -y python3-pip - this might not be needed 
# pip3 install smbus2
# source /opt/ros/eloquent/setup.bash
# im gonna comment line by line for some of this for my own and likely team member's sanity, this feature is probably getting scraped if this doesnt work perfectly

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from smbus2 import SMBus

I2C_BUS = 1
# Set whichever you have:
USE_INA219 = True  # set False if using ADS1115
INA219_ADDR = 0x40
ADS1115_ADDR = 0x48

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.bus = SMBus(I2C_BUS)
        self.timer = self.create_timer(1.0, self.tick)  # 1 Hz or 1 second
        self.voltage_divider_ratio = 3.3  # <-- TUNE THIS (e.g., 3.3 or 4.0)

    def read_voltage_ina219(self):  
        raw = self.bus.read_word_data(INA219_ADDR, 0x02) # reads a 16 but register that stores the bus voltage
        raw = ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF) # swaps the bytes as its read backwards
        bus_voltage_lsb = 0.004  # 4 mV scale
        bus_v = ((raw >> 3) * bus_voltage_lsb) # shifts right 2 bits since we dont care about status bits, multiply by scale
        return bus_v * self.voltage_divider_ratio # the board doesnt send full raw voltage to the chip, so this scales the number we get to guess the real voltage
                                                  # this alone is why we're probably scrapping this feature, for at least the jetson

    def read_voltage_ads1115(self): #this is for a different chip address or something 
        self.bus.write_i2c_block_data(ADS1115_ADDR, 0x01, [0xC1, 0x83]) # prepare ADC config register
        time.sleep(0.01) #wait for conversion
        data = self.bus.read_i2c_block_data(ADS1115_ADDR, 0x00, 2) #read 2 bytes from the reigster, this is the raw ADC output
        raw = (data[0] << 8) | data[1] #combine the bytes
        if raw & 0x8000:  
            raw -= 1 << 16 #two's complement
        v = raw * 0.000125 #for adc, they do 125 microvolts
        return abs(v) * self.voltage_divider_ratio #same as last method

    def voltage_to_percentage(self, v):
        # Assume a 3S Li-ion pack (nominal 11.1V). Rough mapping.
        # Full ~12.6V, empty (under load) ~10.8V. Clamp to [0,100].
        v_full, v_empty = 12.6, 10.8
        pct = (v - v_empty) / (v_full - v_empty) * 100.0
        return max(0.0, min(100.0, pct))

    def tick(self):
        try:
            if USE_INA219:
                pack_v = self.read_voltage_ina219()
            else:
                pack_v = self.read_voltage_ads1115()
        except Exception as e:
            self.get_logger().warn(f'I2C read failed: {e}')
            return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(pack_v)
        msg.current = float('nan')  # set if you wire shunt/current
        msg.percentage = self.voltage_to_percentage(pack_v) / 100.0
        msg.present = True
        self.pub.publish(msg)
        self.get_logger().info(f'Battery: {pack_v:.2f} V ({msg.percentage*100:.0f}%)')

def main():
    rclpy.init()
    node = BatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()