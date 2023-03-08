#!/usr/bin/env python3

from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout,QHBoxLayout, QLabel, QSlider, QLineEdit, QComboBox
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QFont
from threading import Thread
import signal
from numpy import pi
from scipy.spatial.transform import Rotation
import rclpy
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

font = QFont("Helvetica", 9, QFont.Bold)


class Slider:
    range = 1000

    def __init__(self, legend, bound, vlayout):
        self.bound = bound
        
        layout = QHBoxLayout()
        
        label = QLabel(legend)
        label.setFont(font)
        layout.addWidget(label)
        
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setFont(font)
        self.slider.setRange(0, self.range)
        self.slider.setValue(self.range//2)
        layout.addWidget(self.slider)
        self.slider.valueChanged.connect(self.onValueChanged)
        
        self.display = QLineEdit('0')
        self.display.setAlignment(Qt.AlignRight)
        self.display.setFont(font)
        self.display.setReadOnly(True)
        layout.addWidget(self.display)
        
        vlayout.addLayout(layout)
        
    def onValueChanged(self, event):
        self.display.setText(f'{round(self.value(), 2)}')
                
    def value(self):
        return -self.bound + self.slider.value()*2*self.bound/self.range


class CamViewPublisher(QWidget):
    def __init__(self):
        super(CamViewPublisher, self).__init__()
        rclpy.init(args=None)
        self.node = rclpy.create_node('cam_view')
        
        self.tf_buffer = Buffer()
        self.br = TransformBroadcaster(self.node)
        self.tf = TransformListener(self.tf_buffer, self.node)
        self.transform = TransformStamped()
        self.transform.child_frame_id = 'coral_cam_view'
        
        # build layout
        self.vlayout = QVBoxLayout(self)
        
        # reference link
        link_layout = QHBoxLayout()
        label = QLabel('Target frame')
        label.setFont(font)
        link_layout.addWidget(label)
        self.links = QComboBox()
        link_layout.addWidget(self.links)
        self.vlayout.addLayout(link_layout)
        self.avail_frames = []
        
        # poses
        self.axes = {}
        for axis in ('x','y','z','roll','pitch','yaw'):
            bound = 4 if len(axis) == 1 else pi
            self.axes[axis] = Slider(axis, bound, self.vlayout)
                
        self.running = True
        
    def update_frames(self):
        all_frames = [''] + sorted(frame.split()[1] for frame in self.tf_buffer.all_frames_as_string().splitlines())
        if self.transform.child_frame_id in all_frames:
            all_frames.remove(self.transform.child_frame_id)
            
        if all_frames != self.avail_frames:
            self.avail_frames = all_frames
            link = self.links.currentText()
            idx = self.avail_frames.index(link)
            self.links.clear()
            self.links.addItems(all_frames)
            self.links.setCurrentIndex(idx)
    
    def publish(self):
        self.update_frames()
        link = self.links.currentText()
        if link == '':
            return
                
        R = Rotation.from_euler('xyz',[self.axes[axis].value() for axis in ('roll','pitch','yaw')]).inv()
        
        t = R.apply([self.axes[axis].value() for axis in 'xyz'])
        
        for i,axis in enumerate('xyz'):
            setattr(self.transform.transform.translation, axis, t[i])
            
        q = R.as_quat()
        for i,axis in enumerate('xyzw'):
            setattr(self.transform.transform.rotation, axis, q[i])
            
        self.transform.header.frame_id = link
        self.transform.header.stamp = self.node.get_clock().now().to_msg()
        self.br.sendTransform(self.transform)

    def loop(self):
        while self.running and rclpy.ok():
            self.publish()
            rclpy.spin_once(self.node, timeout_sec=0.1)
                        
        if self.running:
            self.node.destroy_node()
            rclpy.shutdown()
                        

if __name__ == '__main__':
    app = QApplication(['cam_view'])
    cam = CamViewPublisher()
    
    try:
        Thread(target=cam.loop).start()
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        cam.show()
        app.exec_()
        cam.running = False
    except:
        cam.node.destroy_node()
        rclpy.shutdown()
        cam.running = False
