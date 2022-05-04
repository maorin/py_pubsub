# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import os
import numpy as np
from chameleon.config import TRUE




# 2336x1080 屏幕长宽
# 1168x 540 中心点
# 手动测试 50有反应
#ros@ros:~$ adb shell input swipe 2335 1  2300 50 50

def sildes(x, y, x1, y1, h):
    os.system('adb shell input swipe {} {} {} {} {}'.format(x, y, x1, y1, h))
    

#判断中心点在不在矩形内  
class MinimalSubscriber(Node):
    # 1168x 540 中心点 
    #"[0.92,0.71,0.98,0.73]" 给的是百分比
    #TODO 改成 0.5 0.5为百分比 中心点 更好一些 给出瞄准心的百分比坐标
    def point_inside_rect(self, data):
        d = data[1:-1].split(",")
        rect = [2336 * float(d[0]), 1080 * float(d[1]), 2336 * float(d[2]), 1080 * float(d[3])]
        print(rect)
        if (rect[0] < 1168 < rect[2]) and (rect[1] < 540 < rect[3]):
            print("ready go")
            return TRUE
        print("no ready!  touch screen!")
        return False

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        
        #判断中心点 是否在矩形内
        self.point_inside_rect(msg.data)
        
        
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
