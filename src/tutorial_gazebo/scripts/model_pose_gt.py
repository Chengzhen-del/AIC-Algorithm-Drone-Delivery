#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Twist

pose_publishers = {}
twist_publishers = {}
down_sample = 1
idx = -1
latch = False

def callback(model_states):
    global idx
    global pose_publishers
    global twist_publishers
    global down_sample
    global latch
    idx = (idx + 1) % down_sample
    if idx != 0:
        return
    for i, name in enumerate(model_states.name):
        if name not in pose_publishers:
            pose_publishers[name] = rospy.Publisher(f'/ground_truth/{name}/pose', PoseStamped, queue_size=1, latch=latch)
            twist_publishers[name] = rospy.Publisher(f'/ground_truth/{name}/twist', Twist, queue_size=1, latch=latch)
        p = PoseStamped()
        p.pose = model_states.pose[i]
        p.header.frame_id = 'map'
        p.header.stamp = rospy.Time.now()
        pose_publishers[name].publish(p)
        twist_publishers[name].publish(model_states.twist[i])

def main():
    global down_sample
    global latch
    rospy.init_node('model_pose_gt')
    down_sample = rospy.get_param('~down_sample', 1)
    latch = rospy.get_param('~latch', False)
    sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
