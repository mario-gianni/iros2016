clear all
close all

filename1 = '20160225171559_seqA';
filename2 = '20160225171559_seqB';
filename3 = '20160225171559_seqc';

load(['dataset_g/' filename1 '.mat']);

addpath(genpath('/home/alcor/machine_learning/multi_task_learning_gp'));
addpath(genpath('/home/alcor/machine_learning/gaussian_processes'));


%% 1. Generating samples
p = 0.8;
[ xtrain, ytrain,x,Y, M, D, nx, ind_kf_train, ind_kx_train ] = split_data_set(state,cmd_vel,p);

%% 2. Assigns cell data for learning and prediction
covfunc_x = {'covSEard'};
%covfunc_x = {'covSum', {'covSEard','covNoise'}};
irank = M; % rank for Kf (1, ... M). irank=M -> Full rank
data  = {covfunc_x, xtrain, ytrain, M, irank, nx, ind_kf_train, ind_kx_train};

%% 3. Hyper-parameter learning
[logtheta_all,deriv_range] = init_mtgp_default(xtrain, covfunc_x, M, irank);
disp('Learning');
[logtheta_all,nl] = learn_mtgp(logtheta_all, deriv_range, data);

plot(nl,'linewidt',3); 
title('Negative Marginal log-likelihood');

%% 4. Making predictions at all points on all tasks
disp('Prediction');
[ Ypred, Vpred ] = predict_mtgp_all_tasks(logtheta_all, data, x );
disp('Finished');

%% 5. Component-wise Euclidean Distance evaluation
[n,m] = size(Y);
error = zeros(n,1);
for i = 1 : M
    error_c = (Y(:,i) - Ypred(:,i)).^2;
    error = error + error_c; 
end
error_final = sqrt(error);
plot(error_final,'linewidt',3); 
title('Component-wise Euclidean Distance');

mse = norm((Y - Ypred).^2);
fprintf('Froebenius Norm: %15.5f\n',mse);

