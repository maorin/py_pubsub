# py_pubsub
https://docs.ros.org/en/rolling/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html#

colcon build --packages-select py_pubsub

colcon build --symlink-install --packages-ignore --packages-select py_pubsub

colcon build --symlink-install

ros2 run py_pubsub talker


ros2 run py_pubsub listener