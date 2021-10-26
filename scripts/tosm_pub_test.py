#!/usr/bin/env python3
from tosm_database_handler import TOSMDatabaseHandler
from tosm_database_sparql import TOSMDatabaseSPARQL
from ses_map_msgs.msg import SESMap, Object, Place
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, PolygonArray
import rospy
import rospkg
import re

ses_map_pub = rospy.Publisher('SESMap', SESMap, queue_size=5)
ses_place_pub = rospy.Publisher('/current_place', Place, queue_size=5)

if __name__ == "__main__":
    rospy.init_node("pub_test")

    print()
    print("*****TOSM Visualization Node")
    print()

    tosm = TOSMDatabaseHandler(rospy.get_param("owl_file_name"), rospy.get_param("owl_file_path"))

    test = SESMap()
    obj_test = Object()
    obj_test.header.frame_id = "map"
    obj_test.object_name = "hingeddoor116"
    obj_test.ID = 116

    test.objects.append(obj_test)

    obj_test3 = Object()
    obj_test3.header.frame_id = "map"
    obj_test3.object_name = "hingeddoor116"
    obj_test3.ID = 116

    test.objects.append(obj_test3)

    # obj_test2 = Object()
    # obj_test2.header.frame_id = "map"
    # obj_test2.object_name = "hingeddoor18"
    # obj_test2.ID = 18

    # test.objects.append(obj_test2)

    ses_map_pub.publish(test)
    
    test = Place()
    test.header.frame_id = 'map'
    test.place_name = 'corridor15'
    test.ID = 15
    
    ses_place_pub.publish(test)

