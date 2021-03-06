## festo_ovem
# Package Summary
This package contains a ROS driver node for Festo OVEM vacuum generator. 
The package uses the iolink package for communications.
- Author: Torfi Thorhallsson - torfith@ru.is
- Licence: BSD
- Source: git [https://github.com/torfith/festo_ovem](https://github.com/torfith/festo_ovem)
#  Supported Devices
- Festo OVEM-10-H-B-QOCE-N-LK 
# Nodes
## festo_ovem_node
ROS driver for the Festo OVEM vacuum generator.
### Published topics
- vacuum/is_out_a (std_msgs:Bool) 
- vacuum/is_out_b (std_msgs:Bool) 
- vacuum/pressure (std_msgs:Float32)  
Absolute pressure: -1...0 bar
### Subscribed topics
- vacuum/set_ejection_on (std_msgs:Bool) 
- vacuum/set_suction_on (std_msgs:Bool) 
### Parameters
- interface_name (string)
## Command-line tools
### Launching node
> $ roslaunch festo_ovem festo_ovem.launch [interface_name="enp3s0"]
### Simple operation 
> $ rosrun festo_ovem pick  
> $ rosrun festo_ovem drop
## festo_ovem_test
> $ sudo ~/catkin_ws/devel/lib/festo_ovem/festo_ovem_test enp3s0
## Note
The nodes must be executed with root access to communicate via the EtherCAT interface.
Root access is granted by specifying the launch-prefix="ethercat_grant" inside the launch file.

