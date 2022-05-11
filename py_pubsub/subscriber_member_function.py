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
import json
import base64

from shapely.geometry import LineString
from shapely.geometry import Point

# 2336x1080 屏幕长宽
# 1168x 540 中心点
# 手动测试 50有反应
#ros@ros:~$ adb shell input swipe 2335 1  2300 50 50
HAND_X = 1685
HAND_Y = 319

AIM_X = 1168
AIM_Y = 540

def sildes(x, y, x1, y1, h):
    os.system('adb shell input swipe {} {} {} {} {}'.format(x, y, x1, y1, h))
    

#判断中心点在不在矩形内  
class MinimalSubscriber(Node):
    # 1168x 540 中心点 
    #"[0.92,0.71,0.98,0.73]" 给的是百分比
    #TODO 改成 0.5 0.5为百分比 中心点 更好一些 给出瞄准心的百分比坐标
    def point_inside_rect1(self, data):
        d = data[1:-1].split(",")
        rect = [2336 * float(d[0]), 1080 * float(d[1]), 2336 * float(d[2]), 1080 * float(d[3])]
        print(rect)
        if (rect[0] < 1168 < rect[2]) and (rect[1] < 540 < rect[3]):
            print("ready go")
            return TRUE
        print("no ready!  touch screen!")
        return False


    """
    >>> aa = np.array([1.220,33.33,444.222])
    >>> aa
    array([  1.22 ,  33.33 , 444.222])
    >>> base64.b64encode(aa)
    b'hetRuB6F8z8K16NwPapAQGQ730+Nw3tA'
    >>> cc = base64.b64encode(aa)
    >>> dd = cc.decode("utf-8")
    >>> dd
    'hetRuB6F8z8K16NwPapAQGQ730+Nw3tA'
    >>> ff = dd.encode("utf-8")
    >>> xx = base64.b64decode(ff)
    >>> 
    >>> xx
    b'\x85\xebQ\xb8\x1e\x85\xf3?\n\xd7\xa3p=\xaa@@d;\xdfO\x8d\xc3{@'
    >>> np.frombuffer(xx, dtype=np.float64)
    array([  1.22 ,  33.33 , 444.222])
    >>> 
    """
    #"[[5.62278569e-01 3.41655552e-01 1.00000000e+00 5.13707340e-01\n  1.00000000e-02]\n [1.31348274e+03 3.68987997e+02 2.33600000e+03 5.54803927e+02\n  6.99697971e-01]]"
    def point_inside_rect(self, data):
        ts = data.get("coordinates").encode('utf-8')
        coo = np.frombuffer(base64.b64decode(ts),dtype=np.float64).tolist()
        
        ts1 = data.get("tracks_coordinates").encode('utf-8')
        tcoo = np.frombuffer(base64.b64decode(ts1),dtype=np.float64)
        
        
        print(coo)
        print(type(coo))
        print(tcoo)

        rect = np.array(coo[0:4]) * np.array([2336, 1080, 2236, 1080])
        self.target_point_X = int((rect[2] - rect[0]) / 2)  + rect[0]
        self.target_point_Y = int((rect[3] - rect[1]) / 2)  + rect[1]
        
        print(rect)
        if (rect[0] < 1168 < rect[2]) and (rect[1] < 540 < rect[3]):
            print("ready go")
            return TRUE
        print("no ready!  touch screen!")
        return False


        return False


    def find_target_enemy(self, data):
        res = json.loads(data)
        
        
        if self.target_enemy:
            target_id = self.target_enemy.get("id")
            for d in res:
                if d.get("id") == target_id:
                    print("@@@@@@@找到目标 %s" % d)
                    return d
            
            print("!!!!!!!!!!!!!失去目标！！！！！！！！！！！！！")
        print("目标第一次被确认 %s" % res[0])
        return res[0]
            

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.target_enemy = {}
        self.target_data = ""
        self.target_point_X = None
        self.target_point_Y = None
        self.done = False
        
    def open_fire(self):
        pass
    
    """
    2D游戏 瞄准功能
    已知 
       1. 手点的位置 固定 (1685, 319)
       2. 准点 (1168, 540)
       3. 目标  目标检测中心点
       
    已知三个点 求平行4边形的第4个点  这样会有3个平行4边形。
    排除2个  排除方法  因为准心X轴点  一直小于 手点 所以只有两个可能 


        point4.x = point1.x + (point3.x - point2.x) 
        point4.y = point1.y + (point3.y - point2.y) 
        顺便说一句，给定 3 个点，您可以构建三个平行四边形，另外两个由下式给出
        
        point4.x = point2.x + (point1.x - point3.x) 
        point4.y = point2.y + (point1.y - point3.y) 
        和
        
        point4.x = point3.x + (point2.x - point1.x) 
        point4.y = point3.y + (point2.y - point1.y) 



        from shapely.geometry import LineString
        from shapely.geometry import Point
        
        p = Point(5,5)
        c = p.buffer(3).boundary
        l = LineString([(0,0), (10, 10)])
        i = c.intersection(l)
        
        print i.geoms[0].coords[0]
        (2.8786796564403576, 2.8786796564403576)
        
        print i.geoms[1].coords[0]
        (7.121320343559642, 7.121320343559642)

        如果有一个交点直接
        >>> i.coords[0]
        (7.121320343559642, 7.121320343559642)


       第一条线：  手点的准置到准心 
      第二条线： 准心到目标。
      求手和准心到目标的平行线上的点 优化距离和速度
    """
    def aim_target(self):
        #如果目标在右边  点要大
        if self.target_point_X > AIM_X:
            line_x = HAND_X + (self.target_point_X - AIM_X)
        else:
            line_x = HAND_X - (AIM_X - self.target_point_X)
            
        if self.target_point_Y > AIM_Y:
            line_y = HAND_Y + (self.target_point_Y - AIM_Y)
        else:
            line_y = HAND_Y - (AIM_Y - self.target_point_Y)
            
        print("目标点是：(%s %s) 手要移动的点：(%s %s)" % (self.target_point_X, self.target_point_Y, line_x, line_y))
        p = Point(HAND_X,HAND_Y)
        
        #TODO 换算DPI 移动的距离
        c = p.buffer(30).boundary
        l = LineString([(HAND_X,HAND_Y), (line_x, line_y)])
        i = c.intersection(l)
        print("xxxxxxxxxxxxxxxxxxxxxx  goal: X: %s  Y: %s" % (int(i.coords[0][0]), int(i.coords[0][1])))
        sildes(HAND_X, HAND_Y, int(i.coords[0][0]), int(i.coords[0][1]), 50)
        
        self.done = True
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        
        #查找目标敌人
        self.target_enemy = self.find_target_enemy(msg.data)
        
        #判断中心点 是否在矩形内
        if self.point_inside_rect(self.target_enemy):
            self.open_fire()
        else:
            self.aim_target()       

        
        


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
