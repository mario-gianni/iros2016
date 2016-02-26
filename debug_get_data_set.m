
%% debug
% check recorded data consistency 

%% debug robot state
% labels{ 3}='tf_base_link_world.getOrigin().getX()';
% labels{ 4}='tf_base_link_world.getOrigin().getY()';
% labels{ 5}='tf_base_link_world.getOrigin().getZ()';  
% labels{ 6}='tf_base_link_world.getRotation().getX()';  
% labels{ 7}='tf_base_link_world.getRotation().getY()';  
% labels{ 8}='tf_base_link_world.getRotation().getZ()';  
% labels{ 9}='tf_base_link_world.getRotation().getW()';

% extract yaw from quaternion in tf
i_robot_quat = [6,7,8,9];
robot_quat_b_w = D(:,i_robot_quat); % world wrt base
[n,m] = size(robot_quat_b_w);
robot_euler_w_b = zeros(n,3);
for i = 1 : n
    robot_quat_w_b = quatinv([robot_quat_b_w(i,4),robot_quat_b_w(i,1:3)]);
    robot_euler_w_b(i,:) = quat2angle(robot_quat_w_b);%,'ZYX'); % explicited convention for ZYX = [yaw, pitch , roll]
end
robot_yaw_w_b = robot_euler_w_b(:,1);

% extract position from tf 
i_robot_pos_b_w_tf = [3,4,5];
robot_pos_b_w_tf = D(:,i_robot_pos_b_w_tf); % world wrt base
%T_b_w_tf = zeros(4,4,n);
robot_pos_w_b_tf = zeros(n,3);
for i=1:n
    R_b_w = rotFromQuat(robot_quat_b_w(i,:));
    %T_b_w_tf(:,:,i) = [ R_b_w , robot_pos_b_w_tf(i,:)'; [0 0 0], 1];
    R_w_b = R_b_w'; 
    robot_pos_w_b_tf(i,:) = -R_w_b * robot_pos_b_w_tf(i,:)';
end

%% debug timestamps 

% labels{ 1}='ros::Time::now().toNSec()';
% labels{ 2}='vicon_base->header.stamp.toNSec()';
% labels{10}='tracks_cmd->header.stamp.toNSec()';
% labels{67}='state->header.stamp';
i_timestamps = [1,2,10,67];
timestamps = D(:,i_timestamps); % nanoseconds 
%timestamps = timestamps*1e-9 - timestamps(1,1)*1e-9; 


%% plot data 

%plot robot yaw angle w_b
errorYaw = zeros(n,1);
for i=1:n
    errorYaw(i) = diffS1(robot_yaw_w_b(i),robot_pos_o(i,3));
end
figure
hold on; 
grid on; 
title('yaw error')
plot(errorYaw)


% plot robot position 
figure
hold on; 
grid on; 
axis equal; 
title('xy trasjectory')
plot(robot_pos(:,1),robot_pos(:,2),'b.');
plot(robot_pos_w_b_tf(:,1),robot_pos_w_b_tf(:,2),'r.');
legend('pos','postf')

% plot timestamps
figure
hold on
grid on
title('timestamps')
colorStyles = ['b.','r.','k.','g.'];
for i=1:1%length(colorStyles)
    plot(timestamps(:,i),colorStyles(i));
end
legend('ros::Time::now().toNSec()','vicon_base->header.stamp.toNSec()',...
    'tracks_cmd->header.stamp.toNSec()','state->header.stamp')
