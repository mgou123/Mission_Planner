# behavior_tree

Defines the behavior tree engine and default plugin models compatible with ```BehaviorTree.CPP```, for use in ROS 1.

## Dependencies

Depends on ```BehaviorTree.CPP```, install from [here](https://github.com/BehaviorTree/BehaviorTree.CPP).

## Adding a new default plugin

1. Create the plugin header in the correct   ```include/behavior_tree``` folder (action, condition, control or decorator).
2. Create the relevant cpp file in the ```plugins``` folder until the correct sub-folder (should match your header sub-folder).
3. Add as a C++ library in CMakeLists.txt (follow the format of the other plugin models).
3. Update bt_nodes.xml with your new model definition for Groot.
4. If using in a behavior tree, make sure to update the mission_planner params ```config``` file. If the model is going to be used repeatedly, you can add it into the
   default plugin libs in ```asv_mission_planner/src/mission_planner.cpp``` in plugin_libs in the ```MissionPlanner``` constructor.

To create custom plugins, create a new package called ```{name}_bt_plugins``` with an bt_nodes.xml file. Follow the structure of the ```plugins``` folders in this package.