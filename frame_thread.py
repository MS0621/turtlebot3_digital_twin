from PyQt5.QtCore import QThread, pyqtSignal, QObject
import numpy as np

class FrameThread(QThread):
    update_main = pyqtSignal(np.ndarray)
    update_yolo = pyqtSignal(np.ndarray)
    update_lane = pyqtSignal(np.ndarray)

    def __init__(self, widget):
        super().__init__()
        self.widget = widget
        self.running = True

    def run(self):
        while self.running:
            if self.widget.main_frame is not None:
                self.update_main.emit(self.widget.main_frame.copy())
            if self.widget.yolo_frame is not None:
                self.update_yolo.emit(self.widget.yolo_frame.copy())
            if self.widget.lane_frame is not None:
                self.update_lane.emit(self.widget.lane_frame.copy())
            self.msleep(99)  # 15fps 정도

        
    def stop(self):
        self.running = False
        self.wait()

