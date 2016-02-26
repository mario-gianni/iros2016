clear all
close all

%% open files and extract all data 

filename1 = '20160225171559_seqA';
filename2 = '20160225171559_seqB';
filename3 = '20160225171559_seqC';

d1 = csvread(['csvdata/' filename1 '.csv']);
d2 = csvread(['csvdata/' filename2 '.csv']);
d3 = csvread(['csvdata/' filename3 '.csv']);

D = d1;

%% extract needed observations 

% labels{68}='state->x';
% labels{69}='state->y';
% labels{70}='state->yaw';
% labels{71}='state->xD';
% labels{72}='state->yD';
% labels{73}='state->yawD'


i_robot_pos = [68,69,70]; 
i_robot_pos_deriv = [71,72,73];

robot_pos = D(:,i_robot_pos); % [x,y,yaw]
robot_pos_deriv = D(:,i_robot_pos_deriv);
[n,m] = size(robot_pos_deriv);

d = 0.1;
robot_pos_o = zeros(n,m); 
robot_pos_o_deriv = zeros(n,m);

% [xo, yo, yawo] = [x + d*cos(yaw),  y + d*sin(yaw) ,yaw]
robot_pos_o(:,1) = robot_pos(:,1) + d*cos(robot_pos(:,3));
robot_pos_o(:,2) = robot_pos(:,2) + d*sin(robot_pos(:,3));
robot_pos_o(:,3) = robot_pos(:,3);

% [xoD, yoD, yawoD] = [xD -d*sin(yaw)*yawD , yD +d*cos(yaw)*yawD, yawD ] 
robot_pos_o_deriv(:,1) =  robot_pos_deriv(:,1) - d*sin(robot_pos(:,3)).* robot_pos_deriv(:,3); 
robot_pos_o_deriv(:,2) =  robot_pos_deriv(:,2) + d*cos(robot_pos(:,3)).* robot_pos_deriv(:,3); 
robot_pos_o_deriv(:,3) =  robot_pos_deriv(:,3);


% robot_deriv = zeros((n-1),2*m);
% robot_deriv(:,1:m) = robot_pos_deriv(1:(n-1),:);
% robot_deriv(:,(m+1):end) = robot_pos_deriv(2:n,:);

robot_deriv_w_b_k = robot_pos_o_deriv(1:(n-1),:);
robot_deriv_w_b_kp1 = robot_pos_o_deriv(2:n,:);

%labels{11}='tracks_cmd->left';
%labels{12}='tracks_cmd->right';
i_vel = [11,12];

% data set for estimating g function
input_g = [robot_deriv_w_b_k,robot_deriv_w_b_kp1,robot_pos_o(1:(end-1),3)]; % [qD_k+1, qD_k, q_k]
output_g = D(1:(end-1),i_vel); % [vl_vr_k]

% data set for estimating f function
input_f = [robot_deriv_w_b_k,robot_pos_o(1:(end-1),3),output_g]; % [qD_k, q_k, vl_vr_k]
output_f = robot_deriv_w_b_kp1; % [qD_k+1]

save(['dataset_g/' filename1 '.mat'],'input_f','output_f','input_g','output_g')

%% debug 
%debug_get_data_set
