""" ROS RGB image publisher for DIGIT sensor """

import hydra
from cv_bridge import CvBridge
from pathlib import Path
import rospy
from sensor_msgs.msg import CompressedImage
from digit_depth.digit.digit_sensor import DigitSensor
import os
import hydra
from omegaconf import OmegaConf
from hydra import initialize, compose

relative_config_path = os.path.join('..','config')

class ImageFeature:
    def __init__(self):
        self.image_pub = rospy.Publisher("/digit/rgb/image_raw/compressed",
                                         CompressedImage, queue_size=10)
        self.br = CvBridge()

def rgb_pub():
    config_file_name = rospy.get_param("~config_file_name", "digit_D20951.yaml")

    print(f"[DIGIT] Using config file: {config_file_name}")

    with initialize(config_path=relative_config_path):
        cfg = compose(config_name=config_file_name)
    
    digit_sensor = DigitSensor(cfg.sensor.fps, "QVGA", cfg.sensor.serial_num)
    ic = ImageFeature()
    rospy.init_node('image_feature', anonymous=True)
    digit_call = digit_sensor()
    br = CvBridge()
    while True:
        frame = digit_call.get_frame()
        msg = br.cv2_to_compressed_imgmsg(frame, "png")
        msg.header.stamp = rospy.Time.now()
        ic.image_pub.publish(msg)
        # rospy.loginfo("Published image")

if __name__ == "__main__":
    rgb_pub()