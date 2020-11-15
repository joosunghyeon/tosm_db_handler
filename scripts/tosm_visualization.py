#!/usr/bin/env python3
from tosm_database_handler import TOSMDatabaseHandler
from tosm_database_sparql import TOSMDatabaseSPARQL
from ses_map_msgs.msg import SESMap, Object, Place
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, PolygonArray
import rospy
import rospkg
import re

ses_map_object_vis_pub = rospy.Publisher('SESMap_object_vis', BoundingBoxArray, queue_size=5)
ses_map_place_vis_pub = rospy.Publisher('SESMap_place_vis', PolygonArray, queue_size=5)

def vis_callback(msg):
    obj_bba = BoundingBoxArray()
    obj_bba.header.frame_id = 'map'

    for obj in msg.objects:
        obj_bb = BoundingBox()
        res = tosm.query_individual(obj.object_name + str(obj.ID))
        obj_bb.header = obj.header
        pose = res.pose[0].replace('[', '').replace(']', '').split(',')
        size = res.size[0].replace('[', '').replace(']', '').split(',')

        obj_bb.pose.position.x = float(pose[0])
        obj_bb.pose.position.y = float(pose[1])
        obj_bb.pose.position.z = float(pose[2]) + float(size[2])/2.0
        obj_bb.pose.orientation.x = float(pose[3])
        obj_bb.pose.orientation.y = float(pose[4])
        obj_bb.pose.orientation.z = float(pose[5])
        obj_bb.pose.orientation.w = float(pose[6])

        # bounding box size
        obj_bb.dimensions.x = float(size[0])
        obj_bb.dimensions.y = float(size[1])
        obj_bb.dimensions.z = float(size[2])

        # likelihood
        obj_bb.value = 1

        # determine the color
        if (obj.object_name == "hingeddoor"):
            obj_bb.label = 1
        elif (obj.object_name == "elevatordoor"):
            obj_bb.label = 2
        elif (obj.object_name == "A"):
            obj_bb.label = 3
        elif (obj.object_name == "B"):
            obj_bb.label = 4
        elif (obj.object_name == "C"):
            obj_bb.label = 5
        else:
            obj_bb.label = 10

        obj_bba.boxes.append(obj_bb)

    ses_map_object_vis_pub.publish(obj_bba)

if __name__ == "__main__":
    rospy.init_node("tosm_db_handler")
    rospy.Subscriber("/SESMap", SESMap, vis_callback)

    print()
    print("*****TOSM Visualization Node")
    print()

    tosm = TOSMDatabaseHandler(rospy.get_param("owl_file_name"), rospy.get_param("owl_file_path"))

    rospy.spin()