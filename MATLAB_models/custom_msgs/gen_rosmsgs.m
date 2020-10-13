if ~exist('./geometry_msgs', 'dir')
    display("Error: Geometry msgs package not found. Get the ROS Geometry package from ROS git - https://github.com/ros/common_msgs/tree/noetic-devel/geometry_msgs");
end
% Copy base carla msgs
[status,msg] = copyfile('../../carla_msgs', './carla_msgs');
if status == 0
    display(msg)
    return
end

% Copy Carla ackermann control msgs
[status,msg] = copyfile('../../carla_ackermann_control', './carla_ackermann_control');
if status == 0
    display(msg)
    return
end
% Copy custom msgs
[status,msg] = copyfile('../../custom_msgs', './custom_msgs');
if status == 0
    display(msg)
    return
end

% Generate custom msgs for use with ROS
rosgenmsg(pwd)