clear all
close all

filename1 = '20160225171559_seqA';
filename2 = '20160225171559_seqB';
filename3 = '20160225171559_seqC';

d1 = csvread(['csvdata/' filename1 '.csv']);
d2 = csvread(['csvdata/' filename2 '.csv']);
d3 = csvread(['csvdata/' filename3 '.csv']);

D = d1;

%labels{71}='state->xD';
%labels{72}='state->yD';
%labels{73}='state->yawD'

i_robot_pos_deriv = [71,72,73];
robot_pos_deriv = D(:,i_robot_pos_deriv);
[n,m] = size(robot_pos_deriv);
robot_deriv = zeros(1:(n-1),2*m);
robot_deriv(:,1:m) = robot_pos_deriv(1:(n-1),:);
robot_deriv(:,(m+1):end) = robot_pos_deriv(2:n,:);

%labels{ 6}='tf_base_link_world.getRotation().getX()';  
%labels{ 7}='tf_base_link_world.getRotation().getY()';  
%labels{ 8}='tf_base_link_world.getRotation().getZ()';  
%labels{ 9}='tf_base_link_world.getRotation().getW()';

i_robot_quat = [6,7,8,9];
robot_quat = D(:,i_robot_quat);
[n,m] = size(robot_quat);
robot_euler = zeros(n,3);
for i = 1 : n
    robot_euler(i,:) = quat2angle(robot_quat(i,:));
end
robot_yaw = robot_euler(:,1);

state = [robot_deriv,robot_yaw(1:(end-1))];

%labels{11}='tracks_cmd->left';
%labels{12}='tracks_cmd->right';
i_vel = [11,12];
cmd_vel = D(1:(end-1),i_vel);

save(['dataset_g/' filename1 '.mat'],'state','cmd_vel')
