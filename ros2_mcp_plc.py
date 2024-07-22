import rclpy
from rclpy.node import Node
import pymcprotocol
import time


class PLCCommunicationNode(Node):
    def __init__(self):
        super().__init__('plc_communication_node')
        self.pymc3e = pymcprotocol.Type3E()
        self.pymc3e.connect("192.168.200.10", 3001)
        self.get_logger().info("Connected to PLC")
        
        # Create timers
        self.timer1 = self.create_timer(3.0, self.plc_communication)
        self.timer2 = self.create_timer(0.001, self.plc_communication1)
        self.timer3 = self.create_timer(0.001, self.plc_communication2)

        self.write_data = 1

    def reading_resister_plc(self, resister_name):
        d_val, _ = self.pymc3e.randomread(word_devices=[resister_name], dword_devices=[])
        return d_val[0]

    def write_resister_plc(self, resister_name, write_data):
        self.pymc3e.randomwrite(word_devices=[resister_name], word_values=[write_data], dword_devices=[], dword_values=[])

    def plc_communication(self):
        d_2010_val = self.reading_resister_plc("D2010")
        if self.write_data == 1:
            self.write_resister_plc("D2000", self.write_data)
            self.write_data = 0
        else:
            self.write_resister_plc("D2000", self.write_data)
            self.write_data = 1

    def plc_communication1(self):
        # D2011 (1) = D2005 (0) = D2012 (1) = D2014 (0) 
        # -> D2003 (1) -> D2001 (1) -> D2003 (0) -> D2014 (1) -> D2006 (1) -> D2014 (2) -> D2001 (0)

        d_2011_val = self.reading_resister_plc("D2011")
        d_2005_val = self.reading_resister_plc("D2005")
        d_2012_val = self.reading_resister_plc("D2012")
        d_2014_val = self.reading_resister_plc("D2014")

        if d_2014_val == 2:
            self.get_logger().info("d_2014_val == 2")
            self.get_logger().info(f"{d_2011_val}, {d_2005_val}, {d_2012_val}, {d_2014_val}")
            self.write_resister_plc("D2001", 0)
        elif d_2011_val == 1 and d_2005_val == 0 and d_2012_val == 1 and d_2014_val == 0:
            self.get_logger().info("Found plc_communication1 condition")
            self.write_resister_plc("D2003", 1)
            time.sleep(3)
            self.write_resister_plc("D2001", 1)
            time.sleep(1)
            self.write_resister_plc("D2003", 0)
            if d_2014_val == 1:
                self.write_resister_plc("D2006", 1)

    def plc_communication2(self):
        # D2011 (1) = D2005 (1) = D2013 (1) = D2015 (0) 
        # -> D2004 (1) -> D2002 (1) -> D2004 (0) -> D2015 (1) -> D2007 (1) -> D2015 (2) -> D2002 (0)

        d_2011_val = self.reading_resister_plc("D2011")
        d_2005_val = self.reading_resister_plc("D2005")
        d_2013_val = self.reading_resister_plc("D2013")
        d_2015_val = self.reading_resister_plc("D2015")

        if d_2015_val == 2:
            self.get_logger().info(f"d_2015_val == {d_2015_val}")
            self.write_resister_plc("D2002", 0)
        elif d_2011_val == 1 and d_2005_val == 1 and d_2013_val == 1 and d_2015_val == 0:
            self.get_logger().info("Found plc_communication2 condition")
            self.write_resister_plc("D2004", 1)
            time.sleep(3)
            self.write_resister_plc("D2002", 1)
            time.sleep(1)
            self.write_resister_plc("D2004", 0)
            if d_2015_val == 1:
                self.write_resister_plc("D2007", 1)


def main(args=None):
    rclpy.init(args=args)
    node = PLCCommunicationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
