import sys
import cv2
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QWidget
CAMERA_WIDTH = 1280;CAMERA_HEIGHT = 720 # 设定的相机取样像素参数
class CameraWidget(QWidget):
    def __init__(self, camera_port=5, parent=None):
        super().__init__(parent)
        self.camera = cv2.VideoCapture(camera_port)
        self.camera.set(3, CAMERA_WIDTH)
        self.camera.set(4, CAMERA_HEIGHT)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(50)  # 50ms更新一次

        self.image_label = QLabel(self)
        self.image_label.setFixedSize(340, 240)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setScaledContents(True)

    def update_frame(self):
        ret, frame = self.camera.read()
        if ret:
            image = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(image)
            self.image_label.setPixmap(pixmap)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    camera_widget = CameraWidget()
    camera_widget.show()
    sys.exit(app.exec_())