# ME570-Vasowalla
 ME 570 Robot Motion Planning Coursework


ros2 launch bearing_formation_control multi_turtle.launch.py num_turtles:=5
[INFO] [launch]: All log files can be found below /home/armaan/.ros/log/2024-12-02-08-33-25-305077-ROS2-Fall2025-5083
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [5084]
[INFO] [spawn_turtle.py-2]: process started with pid [5086]
[INFO] [command_executor.py-3]: process started with pid [5088]
[INFO] [logic.py-4]: process started with pid [5090]
[ERROR] [logic.py-4]: process has died [pid 5090, exit code 127, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_1 --params-file /tmp/launch_params_kacb1ivz'].
[INFO] [logic.py-5]: process started with pid [5093]
[ERROR] [logic.py-5]: process has died [pid 5093, exit code 127, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_2 --params-file /tmp/launch_params_pt18ytld'].
[INFO] [logic.py-6]: process started with pid [5095]
[ERROR] [logic.py-6]: process has died [pid 5095, exit code 127, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_3 --params-file /tmp/launch_params_c6043jux'].
[INFO] [logic.py-7]: process started with pid [5097]
[ERROR] [logic.py-7]: process has died [pid 5097, exit code 127, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_4 --params-file /tmp/launch_params_hfobfiy5'].
[INFO] [logic.py-8]: process started with pid [5099]
[ERROR] [logic.py-8]: process has died [pid 5099, exit code 127, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_5 --params-file /tmp/launch_params_1qk7kk3_'].
[INFO] [logic.py-9]: process started with pid [5101]
[ERROR] [logic.py-9]: process has died [pid 5101, exit code 127, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_6 --params-file /tmp/launch_params_an6ibzw_'].
[INFO] [logic.py-10]: process started with pid [5103]
[ERROR] [logic.py-10]: process has died [pid 5103, exit code 127, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_7 --params-file /tmp/launch_params_ai804nxe'].
[INFO] [logic.py-11]: process started with pid [5105]
[ERROR] [logic.py-11]: process has died [pid 5105, exit code 127, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_8 --params-file /tmp/launch_params_f492_tvf'].
[INFO] [logic.py-12]: process started with pid [5107]
[ERROR] [logic.py-12]: process has died [pid 5107, exit code 127, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_9 --params-file /tmp/launch_params_uehhczh3'].
[INFO] [logic.py-13]: process started with pid [5109]
[ERROR] [logic.py-13]: process has died [pid 5109, exit code 127, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/logic.py --ros-args -r __node:=logic_node_10 --params-file /tmp/launch_params_82ywdboj'].
[turtlesim_node-1] Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
[logic.py-4] /usr/bin/env: ‘python3\r’: No such file or directory
[logic.py-5] /usr/bin/env: ‘python3\r’: No such file or directory
[logic.py-6] /usr/bin/env: ‘python3\r’: No such file or directory
[logic.py-7] /usr/bin/env: ‘python3\r’: No such file or directory
[logic.py-8] /usr/bin/env: ‘python3\r’: No such file or directory
[logic.py-9] /usr/bin/env: ‘python3\r’: No such file or directory
[logic.py-10] /usr/bin/env: ‘python3\r’: No such file or directory
[logic.py-11] /usr/bin/env: ‘python3\r’: No such file or directory
[logic.py-12] /usr/bin/env: ‘python3\r’: No such file or directory
[logic.py-13] /usr/bin/env: ‘python3\r’: No such file or directory
[spawn_turtle.py-2] Traceback (most recent call last):
[spawn_turtle.py-2]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/spawn_turtle.py", line 89, in <module>
[spawn_turtle.py-2]     main()
[spawn_turtle.py-2]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/spawn_turtle.py", line 81, in main
[spawn_turtle.py-2]     node = TurtlesSpawnControl(num_turtles)
[spawn_turtle.py-2] NameError: name 'num_turtles' is not defined
[ERROR] [spawn_turtle.py-2]: process has died [pid 5086, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/spawn_turtle.py --ros-args -r __node:=spawn_turtles --params-file /tmp/launch_params__ov4jsdo'].
[turtlesim_node-1] [INFO] [1733146405.997560608] [turtlesim_node]: Starting turtlesim with node name /turtlesim_node
[turtlesim_node-1] [INFO] [1733146406.003688198] [turtlesim_node]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
[command_executor.py-3] Traceback (most recent call last):
[command_executor.py-3]   File "/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/command_executor.py", line 6, in <module>
[command_executor.py-3]     from me570_final_project.msg import command  
[command_executor.py-3] ModuleNotFoundError: No module named 'me570_final_project'
[ERROR] [command_executor.py-3]: process has died [pid 5088, exit code 1, cmd '/home/armaan/me570-final/install/bearing_formation_control/lib/bearing_formation_control/command_executor.py --ros-args -r __node:=command_executor --params-file /tmp/launch_params_rdls37kd'].
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[turtlesim_node-1] [INFO] [1733146427.491042679] [rclcpp]: signal_handler(signum=2)
[INFO] [turtlesim_node-1]: process has finished cleanly [pid 5084]
