#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped


class MocapForwarder:
    def __init__(self):
        self.topic_prefix = self._normalize_topic(
            rospy.get_param("~topic_prefix", "/ground_truth")
        )
        self.vehicle_names = self._load_vehicle_names()
        self.publishers = {}
        self.subscribers = {}

        rospy.loginfo(
            "mocap_forward topic_prefix=%s vehicle_names=%s",
            self.topic_prefix,
            self.vehicle_names,
        )
        self.setup_forwarding()

    @staticmethod
    def _normalize_topic(topic):
        if not topic:
            return "/"
        topic = topic.strip()
        if not topic.startswith("/"):
            topic = "/" + topic
        if len(topic) > 1:
            topic = topic.rstrip("/")
        return topic

    def _load_vehicle_names(self):
        vehicle_names = rospy.get_param("~vehicle_names", [])

        if isinstance(vehicle_names, str):
            vehicle_names = [
                item.strip() for item in vehicle_names.split(",") if item.strip()
            ]

        if not isinstance(vehicle_names, list):
            raise rospy.ROSException("~vehicle_names must be a list or comma-separated string")

        normalized_names = []
        for vehicle_name in vehicle_names:
            if not isinstance(vehicle_name, str):
                raise rospy.ROSException("~vehicle_names entries must be strings")
            vehicle_name = vehicle_name.strip()
            if vehicle_name:
                normalized_names.append(vehicle_name)

        return normalized_names

    def setup_forwarding(self):
        for vehicle_name in self.vehicle_names:
            source_topic = f"{self.topic_prefix}/{vehicle_name}/pose"
            target_topic = f"/{vehicle_name}/mavros/vision_pose/pose"
            publisher = rospy.Publisher(target_topic, PoseStamped, queue_size=10)
            subscriber = rospy.Subscriber(
                source_topic,
                PoseStamped,
                self._make_callback(vehicle_name),
                queue_size=10,
            )
            self.publishers[vehicle_name] = publisher
            self.subscribers[vehicle_name] = subscriber
            rospy.loginfo("forward %s -> %s", source_topic, target_topic)

    def _make_callback(self, vehicle_name):
        def callback(msg):
            self.publishers[vehicle_name].publish(msg)

        return callback


def main():
    rospy.init_node("mocap_forward")
    MocapForwarder()
    rospy.spin()


if __name__ == "__main__":
    main()

