# ME570-Vasowalla
 ME 570 Robot Motion Planning Coursework


ros2 launch bearing_formation_control multi_turtle.launch.py
[INFO] [launch]: All log files can be found below /home/armaan/.ros/log/2024-12-02-09-33-45-137101-ROS2-Fall2025-8955
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [8956]
[INFO] [spawn_turtle.py-2]: process started with pid [8958]
[INFO] [command_executor.py-3]: process started with pid [8960]
[INFO] [logic.py-4]: process started with pid [8962]
[INFO] [logic.py-5]: process started with pid [8964]
[INFO] [logic.py-6]: process started with pid [8966]
[INFO] [logic.py-7]: process started with pid [8968]
[INFO] [logic.py-8]: process started with pid [8970]
[INFO] [logic.py-9]: process started with pid [8972]
[INFO] [logic.py-10]: process started with pid [8975]
[INFO] [logic.py-11]: process started with pid [8977]
[INFO] [logic.py-12]: process started with pid [8979]
[INFO] [logic.py-13]: process started with pid [8981]
[turtlesim_node-1] Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
[turtlesim_node-1] [INFO] [1733150026.589341973] [turtlesim_node]: Starting turtlesim with node name /turtlesim_node
[turtlesim_node-1] [INFO] [1733150026.629403366] [turtlesim_node]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
[spawn_turtle.py-2] Traceback (most recent call last):
[spawn_turtle.py-2]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/spawn_turtle.py", line 89, in <module>
[spawn_turtle.py-2]     main()
[spawn_turtle.py-2]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/spawn_turtle.py", line 81, in main
[spawn_turtle.py-2]     node = TurtlesSpawnControl(num_turtles)
[spawn_turtle.py-2] NameError: name 'num_turtles' is not defined
[ERROR] [spawn_turtle.py-2]: process has died [pid 8958, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/spawn_turtle.py --ros-args -r __node:=spawn_turtles --params-file /tmp/launch_params_pgge4r9x'].
[logic.py-13] Traceback (most recent call last):
[logic.py-13]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py", line 12, in <module>
[logic.py-13]     from me570_final_project.msg import command, targets, sensordata
[logic.py-13] ModuleNotFoundError: No module named 'me570_final_project'
[logic.py-7] Traceback (most recent call last):
[logic.py-7]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py", line 12, in <module>
[logic.py-7]     from me570_final_project.msg import command, targets, sensordata
[logic.py-7] ModuleNotFoundError: No module named 'me570_final_project'
[ERROR] [logic.py-13]: process has died [pid 8981, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_10 --params-file /tmp/launch_params_p7sply3_'].
[logic.py-6] Traceback (most recent call last):
[logic.py-6]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py", line 12, in <module>
[logic.py-6]     from me570_final_project.msg import command, targets, sensordata
[logic.py-6] ModuleNotFoundError: No module named 'me570_final_project'
[logic.py-8] Traceback (most recent call last):
[logic.py-8]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py", line 12, in <module>
[logic.py-8]     from me570_final_project.msg import command, targets, sensordata
[logic.py-8] ModuleNotFoundError: No module named 'me570_final_project'
[ERROR] [logic.py-7]: process has died [pid 8968, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_4 --params-file /tmp/launch_params_x1evbcqt'].
[logic.py-12] Traceback (most recent call last):
[logic.py-12]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py", line 12, in <module>
[logic.py-12]     from me570_final_project.msg import command, targets, sensordata
[logic.py-12] ModuleNotFoundError: No module named 'me570_final_project'
[ERROR] [logic.py-6]: process has died [pid 8966, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_3 --params-file /tmp/launch_params_juhiftk1'].
[logic.py-5] Traceback (most recent call last):
[logic.py-5]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py", line 12, in <module>
[logic.py-5]     from me570_final_project.msg import command, targets, sensordata
[logic.py-5] ModuleNotFoundError: No module named 'me570_final_project'
[ERROR] [logic.py-8]: process has died [pid 8970, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_5 --params-file /tmp/launch_params_9a2iy9kg'].
[command_executor.py-3] Traceback (most recent call last):
[command_executor.py-3]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/command_executor.py", line 6, in <module>
[command_executor.py-3]     from me570_final_project.msg import command  
[command_executor.py-3] ModuleNotFoundError: No module named 'me570_final_project'
[logic.py-9] Traceback (most recent call last):
[logic.py-9]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py", line 12, in <module>
[logic.py-9]     from me570_final_project.msg import command, targets, sensordata
[logic.py-9] ModuleNotFoundError: No module named 'me570_final_project'
[ERROR] [command_executor.py-3]: process has died [pid 8960, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/command_executor.py --ros-args -r __node:=command_executor --params-file /tmp/launch_params_5kli_g3d'].
[ERROR] [logic.py-9]: process has died [pid 8972, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_6 --params-file /tmp/launch_params_ztbphmv2'].
[logic.py-11] Traceback (most recent call last):
[logic.py-11]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py", line 12, in <module>
[logic.py-11]     from me570_final_project.msg import command, targets, sensordata
[logic.py-11] ModuleNotFoundError: No module named 'me570_final_project'
[logic.py-10] Traceback (most recent call last):
[logic.py-10]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py", line 12, in <module>
[logic.py-10]     from me570_final_project.msg import command, targets, sensordata
[logic.py-10] ModuleNotFoundError: No module named 'me570_final_project'
[ERROR] [logic.py-12]: process has died [pid 8979, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_9 --params-file /tmp/launch_params_st10pv8h'].
[ERROR] [logic.py-11]: process has died [pid 8977, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_8 --params-file /tmp/launch_params_6bwdumyv'].
[ERROR] [logic.py-5]: process has died [pid 8964, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_2 --params-file /tmp/launch_params_t6oojlpx'].
[logic.py-4] Traceback (most recent call last):
[logic.py-4]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py", line 12, in <module>
[logic.py-4]     from me570_final_project.msg import command, targets, sensordata
[logic.py-4] ModuleNotFoundError: No module named 'me570_final_project'
[ERROR] [logic.py-10]: process has died [pid 8975, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_7 --params-file /tmp/launch_params_v00e6llx'].
[ERROR] [logic.py-4]: process has died [pid 8962, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_1 --params-file /tmp/launch_params_bb74qdt0'].
[INFO] [turtlesim_node-1]: process has finished cleanly [pid 8956]
