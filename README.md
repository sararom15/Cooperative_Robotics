# Cooperative Robotics

This  project  implements  a  Task  Priority  Control  on  an  Underwater  Floating  Manipulator  System.   The  code  is  written  in  MATLAB  and  simulated  on  a  provided Unity-based simulation.  

Results are tested on two different UVMs:  Robust (exercises1-4) and Dexrov (exercises 5-6). For the Robust UVM we implemented a control based on 4 consecutive actions: Safe Waypoint Navigation, Vehicle/rock Alignment, Basic Landing Action, the Fixed-Base Manipulation. For the DexROV we implemented 2 of the aforementioned actions (Safe WaypointNavigation  and  Fixed-Base  Manipulation)  and  we  adopted  a  parallel  coordination scheme to split the control of the vehicle and of the arm.
