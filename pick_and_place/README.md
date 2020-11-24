# README for TRINA2 Pick and Place

### Startup

Launch empty world with table/ blocks:
roslaunch trina2_gazebo trina2.launch world_name:=empty_hospital

Call init_blocks once TRINA has gotten into the start position:
rosrun pick_and_place init_blocks 

Start the service node:
roslaunch pick_and_place arm_to_block.launch

### Pick and place ops

Call pick service, specifying which block via argument:
rosservice call /trina2_1/trina_pick "pick_obj: 'unit_box_2'" 

When service returns, TRINA will be "holding" (collision object attached in RViz, but not yet working correctly in Gazebo) the block.

Call place service with (x,y) coordinates on tabletop:
rosservice call /trina2_1/trina_place "x: 0.8 y: 0.4"
  
(x,y) are in TRINA's base link frame.

Reset the objects via a call to init_blocks when appropriate.  



