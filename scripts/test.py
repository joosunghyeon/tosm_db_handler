#!/usr/bin/env python3
from tosm_database_handler import TOSMDatabaseHandler
import rospy
import rospkg

if __name__ == "__main__":
    rospy.init_node("test")

    test = TOSMDatabaseHandler(rospy.get_param("owl_file_name"), rospy.get_param("owl_file_path"))

    # when you find an object (Column)
    # you can compare using query results
    results = test.query_individuals_of_class("Column")    

    for item in results:
        # if matched - True
        if True: 
            # update the pre-registered individuals
            break
    
    # add new individuals (if there is no same object)
    start = rospy.get_rostime().to_sec()
    if True:
        data = {
            "pose" : "[1.0, 2.8, 3.2, 0, 0, 0, 1]",
            "velocity" : "[1, 2, 3, 0, 0, 0]",
            "isKeyObject" : True
        }
        test.add_individual("Column", "101", data)
        data = {
            "pose" : "[11.0, 12.8, 13.2, 0, 0, 0, 1]",
            "velocity" : "[11, 12, 13, 0, 0, 0]",
            "isKeyObject" : True
        }
        test.add_individual("Column", "102", data)
    end = rospy.get_rostime().to_sec()
    print("add individual time: ", format((end-start)*1000, ".3f"), "[ms]")

    # update the individual
    start = rospy.get_rostime().to_sec()
    data = {
        "pose" : "[2.0, 2.0, 3.5, 0, 0, 0, 1]",
        "velocity" : "[10, 20, 30, 0, 0, 0]",
    }
    test.update_individual("Column", "102", data)
    end = rospy.get_rostime().to_sec()
    print("update individual time: ", format((end-start)*1000, ".3f"), "[ms]")

    # delete individual
    #test.delete_individual("column101")

    # save as
    test.save_as(rospy.get_param("save_as_owl_file_name"))
    