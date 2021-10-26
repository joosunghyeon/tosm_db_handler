#!/usr/bin/env python3
from tosm_database_handler import TOSMDatabaseHandler
from tosm_database_sparql import TOSMDatabaseSPARQL
from ses_map_msgs.msg import SESMap, Object, Place
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray, PolygonArray
from geometry_msgs.msg import PolygonStamped, Point32
import rospy
import rospkg
import re
import ast

class TOSMVisualization:
    def __init__(self, tosm, tosm_sq):
        
        rospy.Subscriber("/SESMap", SESMap, self.vis_callback)
        rospy.Subscriber("/current_floor", Place, self.place_callback)
        rospy.Subscriber("/current_place", Place, self.leaf_place_callback)

        self.ses_map_object_vis_pub = rospy.Publisher('SESMap_object_vis', BoundingBoxArray, queue_size=5)
        self.ses_map_place_vis_pub = rospy.Publisher('SESMap_place_vis', PolygonArray, queue_size=5)
        self.detected_object_vis_pub = rospy.Publisher('detected_object_vis', BoundingBoxArray, queue_size=5)
        self.recognized_place_vis_pub = rospy.Publisher('recognized_place_vis', PolygonArray, queue_size=5)

        print()
        print("*****TOSM Visualization Node*****")
        print()

        #self.vis_instances_on_place('floor3')

    def vis_instances_on_place(self, place_name):
        # Bounding box array for objects
        obj_bba = BoundingBoxArray()
        obj_bba.header.frame_id = 'map'
        
        print()
        print('[tosm_visualization]Query objects that is inside of ' + place_name)
        print()
        results_list = tosm_sq.query_objects_insideOf_place(place_name)
        
        for item in results_list:
            s = str(item['s'].toPython())
            s = re.sub(r'.*#',"",s)
            print(s + "  isInsideOf  " + place_name)
            
            # make bounding box msgs to visualize
            obj_bb = BoundingBox()
            
            res = tosm.query_individual(s)
            obj_bb.header.frame_id = 'map'
            obj_bb.header.stamp = rospy.Time.now()
            
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
            if (obj_bb.dimensions.x + obj_bb.dimensions.y + obj_bb.dimensions.z) < 0.01:
                print(s + ' size is 0. Can not visulize')

            # likelihood
            obj_bb.value = 1.0

            # determine the color
            object_name = re.sub(r'[0-9]+', '', res.name)
            obj_bb.label = len(object_name)
            
            obj_bba.boxes.append(obj_bb)

        self.ses_map_object_vis_pub.publish(obj_bba)
        
        print()
        print('[tosm_visualization]Send vis topic for objects is inside of ' + place_name)
        print()
        
        # Polygon array for places
        places_pa = PolygonArray()
        places_pa.header.frame_id = 'map'
        
        print()
        print('[tosm_visualization]Query places that is inside of ' + place_name)
        print()
        results_list = tosm_sq.query_places_insideOf_place(place_name)
        
        for item in results_list:
            s = str(item['s'].toPython())
            s = re.sub(r'.*#',"",s)
            print(s + "  isInsideOf  " + place_name)
        
            ps = PolygonStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = rospy.Time.now()
            
            res = tosm.query_individual(s)
            #boundary = res.boundary[0].replace('[', '').replace(']', '').split(',')
            boundary = ast.literal_eval(res.boundary[0])
            ps.polygon.points = [Point32(v[0],v[1],0) for v in boundary]
            places_pa.polygons.append(ps)
            places_pa.labels.append(len(re.sub(r'[0-9]+', '', res.name)))
            places_pa.likelihood.append(1.0)
            
        self.ses_map_place_vis_pub.publish(places_pa)
        
        print()
        print('[tosm_visualization]Send vis topic for places is inside of ' + place_name)
        print()

    def vis_callback(self, msg):
        # Bounding box array for detected objects
        obj_bba = BoundingBoxArray()
        obj_bba.header.frame_id = 'map'

        for obj in msg.objects:
            res = tosm.query_individual(obj.object_name)
            if res:
                obj_bb = BoundingBox()
                
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
                obj_bb.value = 1.0

                # determine the color
                obj_bb.label = len(obj.object_name)

                obj_bba.boxes.append(obj_bb)
            else:
                print('There are no ' + obj.object_name + '. Add the instance first.')

        self.detected_object_vis_pub.publish(obj_bba)

    def place_callback(self, msg):
        self.vis_instances_on_place(msg.place_name)
        
    def leaf_place_callback(self, msg):
        # Polygon array for the current leaf place
        places_pa = PolygonArray()
        places_pa.header.frame_id = 'map'
        
        res = tosm.query_individual(msg.place_name)
        
        if res:   
            ps = PolygonStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = rospy.Time.now()
            
            boundary = ast.literal_eval(res.boundary[0])
            ps.polygon.points = [Point32(v[0],v[1],0) for v in boundary]
            places_pa.polygons.append(ps)
            places_pa.labels.append(len(re.sub(r'[0-9]+', '', res.name)))
            places_pa.likelihood.append(1.0)
            
            self.recognized_place_vis_pub.publish(places_pa)
        
            print()
            print('[tosm_visualization]The robot is inside of ' + msg.place_name)
            print()
        
        else:
            print('There are no place ' + msg.place_name)
        


if __name__ == "__main__":
    rospy.init_node('tosm_visualizaion')
    tosm = TOSMDatabaseHandler(rospy.get_param("owl_file_name"), rospy.get_param("owl_file_path"))
    tosm_sq = TOSMDatabaseSPARQL(rospy.get_param("owl_file_name"), rospy.get_param("owl_file_path"))
    TOSMVisualization(tosm, tosm_sq)    

    rospy.spin()