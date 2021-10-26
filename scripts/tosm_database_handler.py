#!/usr/bin/env python3
from owlready2 import *

class TOSMDatabaseHandler:
    def __init__(self, owl_file_name, owl_file_path):
        # Load parameters from ROS
        self.owl_file_name = owl_file_name
        self.owl_file_path = owl_file_path
        
        # Load an OWL file and synchronize with a hermit reasoner
        onto_path.append(self.owl_file_path)
        self.onto = get_ontology("file://" + self.owl_file_path + self.owl_file_name).load()
        self.sync_reasoner()
        
        print()
        print("[TOSM Database Handler] Ready to use the database **" + self.owl_file_name)
        print()
        print("INFORMATION")
        print("* number of classes: ", len(list(self.onto.classes())))
        print("* number of individuals: ", len(list(self.onto.individuals())))
        print("* number of object properties: ", len(list(self.onto.object_properties())))
        print("* number of data properties: ", len(list(self.onto.data_properties())))
        print()

    # input: individual_name (string)
    #        ex) "column8"
    # output: individual object
    #         ex) tosm.column8 or None
    def query_individual(self, individual_name):
        # query directly using specific iri
        return self.onto.search_one(iri=self.onto.base_iri + individual_name)

    # input: class_name (string)
    #        ex) "Column"
    # output: individual list
    #         ex) [tosm.column1, tosm.column2] or []
    def query_individuals_of_class(self, class_name):
        # search class to query individuals
        if self.onto.search(iri=self.onto.base_iri + class_name):
            # query database to find instances of the class
            return self.onto.search(type = self.onto[class_name])
        else:
            print("[Error] occurs in query_individuals_of_class. There is no class " + class_name + ". Define first!!")
            return False

    # input: class_name (string)
    #        id (string)
    #        data_properties (Dictionary)
    #        ex) data = {
    #               "pose" : "[1.0, 2.8, 3.2, 0, 0, 0, 1]",
    #               "velocity" : "[1, 2, 3, 0, 0, 0]",
    #               "isKeyObject" : True
    #            }
    #        object_properties (Dictionary)
    #        ex) data = {
    #               "isInsideOf" : 'corridor2',
    #               "isConnectedTo" : 'corridor3, corridor4'
    #            }
    # output: boolean
    def add_individual(self, class_name, id, data_properties, object_properties):
        # search class to make individuals
        if self.onto.search(iri=self.onto.base_iri + class_name):
            # check there is an individual that has same id
            if self.onto.search(iri=self.onto.base_iri + class_name.lower() + id):
                print("[Error] occurs in add_individual. An individual that has *same id* is found. Use update_individual instead.")
                return False
            else:
                indiv_tmp = getattr(self.onto, class_name)(class_name.lower() + id)
                # Set symbolic model (id)
                indiv_tmp.ID.append(int(id))
                
                # Set data properties
                for key in data_properties.keys():
                    # If the data property is defined
                    if self.onto.search(iri=self.onto.base_iri + key):
                        getattr(indiv_tmp, key).append(data_properties[key])
                    else:
                        destroy_entity(indiv_tmp)
                        print("[Error] occurs in add_individual. There is no data property *" + key + ". Define first!!")
                        return False   
                        
                # Set object properties
                for key in object_properties.keys():
                    # If the object property is defined
                    if self.onto.search(iri=self.onto.base_iri + key):
                        s_op = object_properties[key].replace(' ', '').split(',')
                        for op in s_op:
                            res = self.query_individual(op)
                            if res:
                                getattr(indiv_tmp, key).append(res)
                            else:
                                print("[Error] occurs in add_individual. There is no instances * " + op)
                    else:
                        destroy_entity(indiv_tmp)
                        print("[Error] occurs in add_individual. There is no object property *" + key + ". Define first!!")
                        return False
        else:
            print("[Error] occurs in add_individual. There is no class " + class_name + ". Define first!!")
            return False

        return True

    # input: class_name (string)
    #        id (string)
    #        data_properties (Dictionary)
    #        ex) data_properties = {
    #               "pose" : "[2.0, 2.0, 3.5, 0, 0, 0, 1]",
    #               "velocity" : "[10, 20, 30, 0, 0, 0]",
    #            }
    #        object_properties (Dictionary)
    #        ex) data = {
    #               "isInsideOf" : 'corridor2',
    #               "isConnectedTo" : 'corridor3, corridor4'
    #            }
    # output: boolean
    def update_individual(self, class_name, id, data_properties, object_properties=None):
        # search class to update individuals
        if self.onto.search(iri=self.onto.base_iri + class_name):
            # check there is an individual that has same id
            if self.onto.search(iri=self.onto.base_iri + class_name.lower() + id):
                indiv = getattr(self.onto, class_name)(class_name.lower() + id)
                for key in data_properties.keys():
                    # If the data property is defined
                    if self.onto.search(iri=self.onto.base_iri + key):
                        getattr(indiv, key).clear()
                        getattr(indiv, key).append(data_properties[key])
                    else:
                        print("[Error] occurs in update_individual. There is no data property *" + key + ". Define first!!")
                        
                # Update object properties
                if object_properties:
                    for key in object_properties.keys():
                        # If the object property is defined
                        if self.onto.search(iri=self.onto.base_iri + key):
                            getattr(indiv, key).clear()
                            s_op = object_properties[key].replace(' ', '').split(',')
                            for op in s_op:
                                res = self.query_individual(op)
                                if res:
                                    getattr(indiv, key).append(res)
                                else:
                                    print("[Error] occurs in update_individual. There is no instances * " + op)
                        else:
                            print("[Error] occurs in add_individual. There is no object property *" + key + ". Define first!!")
            else:
                print("[Error] occurs in update_individual. An individual that has *same id* is not found. Use add_individual instead.")
                return False
                                
        else:
            print("[Error] occurs in update_individual. There is no class " + class_name + ". Define first!!")
            return False

        return True

    # input: individual_name (string)
    #         ex) "column8"
    # output: boolean
    def delete_individual(self, individual_name):   
        if self.query_individual(individual_name):
            destroy_entity(self.query_individual(individual_name))
            return True
        else:
            print("[Error] occurs in delete_individual. There is no individual " + individual_name + ".")
            return False
        
    def sync_reasoner(self):
        with self.onto:
            sync_reasoner(infer_property_values=True)
        return

    def save(self):
        self.onto.save()
        return

    def save_as(self, file_name):
        print()
        self.onto.save(file = self.owl_file_path + file_name, format = "rdfxml")
        print("[TOSM Database Handler] TOSM database is saved as **" + file_name)
        print()
        return

