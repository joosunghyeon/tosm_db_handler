#!/usr/bin/env python3
from tosm_database_handler import TOSMDatabaseHandler
from tosm_database_sparql import TOSMDatabaseSPARQL
from pyvis.network import Network
import re
import ast

class TOSMVisNetwork:
  def __init__(self):
    print("*****TOSM Vis Network Node*****")
        
    self.tosm = TOSMDatabaseHandler('semantic_map_pohang.owl', '/home/shjoo/catkin_ws/src/tosm_db_handler/tosm_owl/')
    self.tosm_sq = TOSMDatabaseSPARQL('semantic_map_pohang.owl', '/home/shjoo/catkin_ws/src/tosm_db_handler/tosm_owl/')
    self.net = Network(height='750px', width='100%', bgcolor='#222222', font_color='white')  

  def run(self):
    self.set_network()
    self.vis_network()
    
  def set_network(self):    
    # leaf places
    resultsList = self.tosm_sq.query_connected_places()

    for item in resultsList:
      s = str(item['s'].toPython())
      s = re.sub(r'.*#',"",s)
      o = str(item['o'].toPython())
      o = re.sub(r'.*#', "", o)

      self.net.add_node(s, title=s)
      self.net.add_node(o, title=o)
      self.net.add_edge(s, o, weight=.5)
    
    neighbor_map = self.net.get_adj_list()
    for node in self.net.nodes:
      node['title'] += ' is connected to: <br>' + '<br>'.join(neighbor_map[node['id']])
      node['value'] = len(neighbor_map[node['id']])*2
      
    # visualize "inside of" relationships
    self.set_edge_for_inside_of('floor1', 30)
    self.set_edge_for_inside_of('floor2', 30)
    self.set_edge_for_inside_of('floor3', 30)
    self.set_edge_for_inside_of('floor21', 30)
    self.set_edge_for_inside_of('outdoor1', 50)
    self.set_edge_for_inside_of('building1', 50)
    self.set_edge_for_inside_of('building2', 50)
    self.set_edge_for_inside_of('building4', 50)
       
    return
  
  def set_edge_for_inside_of(self, place_name, value):
    resultsList = self.tosm_sq.query_places_insideOf_place(place_name)
    self.net.add_node(place_name, title=place_name, value=value)
    
    for item in resultsList:
      s = str(item['s'].toPython())
      s = re.sub(r'.*#',"",s)
      self.net.add_edge(s, place_name, weight=.1)
      
    return
      
  def vis_network(self):
    
    # self.net.show_buttons(filter_=['nodes'])
    # self.net.show_buttons(filter_=['physics'])
    # self.net.toggle_physics(True)
    
    # setting options for physics and nodes color
    self.net.set_options("""
      var options = {
        "physics": {
          "barnesHut": {
            "gravitationalConstant": -15000,
            "centralGravity": 1.8,
            "springLength": 50,
            "springConstant": 0.05,
            "damping": 0.12,
            "avoidOverlap": 0.08
          }
        },
        "nodes": {
          "color": {
            "border": "rgba(233,124,21,1)",
            "background": "rgba(252,236,81,1)",
            "highlight": {
              "border": "rgba(255,72,80,1)",
              "background": "rgba(255,64,32,1)"
            }
          }
        }
      }
    """)
    
    self.net.show('test.html')
    return

if __name__ == "__main__":
  tvn = TOSMVisNetwork()
  tvn.run()
