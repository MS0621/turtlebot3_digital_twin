from PyQt5.QtCore import QThread
import rclpy
class RosSpinThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.running = True

    def run(self):
        while rclpy.ok() and self.running:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def stop(self):
        self.running = False