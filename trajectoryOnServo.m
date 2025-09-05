rosshutdown();
setenv('ROS_MASTER_URI', 'http://192.168.0.208:11311');
setenv('ROS_IP', '192.168.0.16');
rosinit('192.168.0.208');

% ros publisher
% [jointMessagePublisher, jointMsg] = rospublisher('joint_msgs_array', 'std_msgs/Float32MultiArray');
% gripMessagePublisher = rospublisher('grip_cmd', 'std_msgs/Float32');

servoPub = rospublisher('/servo_cmd', 'std_msgs/Int32');
servoMsg = rosmessage(servoPub);

% loading robot
addpath(genpath(strcat(pwd,'\Dependencies')))
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';


wayPoints = [0.2 -0.2 0.02;0.25 0 0.15; 0.2 0.2 0.02]; % Alternate set of wayPoints
wayPointVels = [0 0 0;0 0.1 0;0 0 0];


numTotalPoints = size(wayPoints,1)*10;
waypointTime = 4;

wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
trajTimes = linspace(0,wpTimes(end),numTotalPoints);
trajectory = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
                     'VelocityBoundaryCondition',wayPointVels');

hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);

% integrating inverse kinematics

ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0 1 1 1];
initialguess = robot.homeConfiguration;


% gripMsg = rosmessage('std_msgs/Float32');
% gripMsg.Data = 30;
% send(gripMessagePublisher,gripMsg);

title('Robot waypoint tracking visualization')
axis([-0.1 0.4 -0.35 0.35 0 0.35]);
for idx = 1:size(trajectory,2)
    tform = trvec2tform(trajectory(:,idx)');
    configSoln(idx,:) = ik('end_effector',tform,weights,initialguess);
    initialguess = configSoln(idx,:);

    
    servoMsg.Data = int32(configSoln(idx,1).JointPosition*180);
    disp(servoMsg.Data);
    send(servoPub, servoMsg);
    show(robot,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
    pause(0.1)
    

end 
% gripMsg.Data = -30;
% send(gripMessagePublisher,gripMsg);
% hold off

