#!/usr/bin/env python3
from tosm_database_handler import TOSMDatabaseHandler
from tosm_database_sparql import TOSMDatabaseSPARQL
from ses_map_msgs.msg import SESMap, Object, Place
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, PolygonArray
import rospy
import rospkg
import re

ses_map_pub = rospy.Publisher('SESMap', SESMap, queue_size=5)

if __name__ == "__main__":
    rospy.init_node("pub_test")

    print()
    print("*****TOSM Visualization Node")
    print()

    tosm = TOSMDatabaseHandler(rospy.get_param("owl_file_name"), rospy.get_param("owl_file_path"))

    test = SESMap()
    obj_test = Object()
    obj_test.header.frame_id = "map"
    obj_test.object_name = "hingeddoor"
    obj_test.ID = 17

    test.objects.append(obj_test)

    obj_test2 = Object()
    obj_test2.header.frame_id = "map"
    obj_test2.object_name = "elevatordoor"
    obj_test2.ID = 316

    test.objects.append(obj_test2)

    ses_map_pub.publish(test)

