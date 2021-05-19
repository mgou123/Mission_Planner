# mp_behavior_tree

Defines the behavior tree engine and default plugin models compatible with ```BehaviorTree.CPP```, for use in ROS 1.
This module provides:
* A C++ template class for integrating ROS actions into Behavior Tree plugins,
* Navigation-specific behavior tree nodes, and
* a generic BehaviorTreeEngine class that simplifies the integration fo BT processing into ROS nodes.

## Dependencies

Depends on ```BehaviorTree.CPP```, install from [here](https://github.com/BehaviorTree/BehaviorTree.CPP).

## Navigation-specific behavior tree nodes

The mp_behavior_tree package provides several navigation-specific nodes that are pre-registered and can be included in Behavior Trees.

## Adding a new default plugin

1. Create the plugin header in the correct   ```include/mp_behavior_tree``` folder (action, condition, control or decorator).
2. Create the relevant cpp file in the ```plugins``` folder until the correct sub-folder (should match your header sub-folder).
3. Add as a C++ library in CMakeLists.txt (follow the format of the other plugin models).
3. Update bt_nodes.xml with your new model definition for Groot.
4. If using in a behavior tree, make sure to update the mission_planner params ```config``` file. If the model is going to be used repeatedly, you can add it into the
   default plugin libs in ```asv_mission_planner/src/mission_planner.cpp``` in plugin_libs in the ```MissionPlanner``` constructor.

To create custom plugins, create a new package called ```{name}_bt_plugins``` with an bt_nodes.xml file. Follow the structure of the ```plugins``` folders in this package.