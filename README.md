# TOSM-based database handler
OWL-based TOSM database handler
    
    # How to test
    roslaunch tosm_db_handler test.launch

# To use python3 in ROS
We use an Owlready2 python3 package to handle the ontology-based database. In order to use python3 in ROS, you should install packages as follow.

    sudo apt-get install python3-pip python3-yaml
    sudo pip3 install rospkg catkin_pkg

# Requirements
Python 3.5, Owlready2 0.23, setuptools 41.0.0

# Version history
Version 0.0.1
- Added database handling APIs (e.g., query individuals, add individuals, update individuals, and save the owl file).
- Added a test python script to explain how to use APIs.
- Todo: Add relationship related APIs such as handling object properties functions.