#!/usr/bin/env python3
from owlready2 import *
import rospy
import rospkg

class TOSMDatabaseComplexQuery:
    def __init__(self):
        rospy.init_node("test")
        file_name = rospy.get_param("owl_file_name")

        my_world = World()
        my_world.get_ontology("file://" + file_name).load()
        sync_reasoner(my_world, infer_property_values=True)
        self.graph = my_world.as_rdflib_graph()

    def query(self):
        
        query = """
            PREFIX tosm: <http://www.semanticweb.org/ses/tosm#>
            SELECT ?o ?s
            WHERE { ?s  tosm:isConnectedTo ?o }
            """

        resultsList = self.graph.query(query)

        #creating json object
        response = []
        for item in resultsList:
            s = str(item['s'].toPython())
            s = re.sub(r'.*#',"",s)

            o = str(item['o'].toPython())
            o = re.sub(r'.*#', "", o)
            response.append({'s' : s, "o" : o})

        print(response) #just to show the output