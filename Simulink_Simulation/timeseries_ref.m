%% import reference signals
% converting array into timesseries type
addpath('ref_sig\')

filename = 'ref_sq_bs_1.mat';
data = load(filename);
time_arr = data.reference(:,1)';
% ts3 = timeseries((1:5)',[0 10 20 30 40]);
% ref_time = timeseries(data.thetas, time_arr);
th1 = [time_arr; data.reference(:,2)'];
th2 = [time_arr; data.reference(:,3)'];

% dth1 = [time_arr; data.reference(:,4)'];
% dth2 = [time_arr; data.reference(:,5)'];

save(".\ref_source\20s\ref_sq_bs_th1_1.mat", "th1");
save(".\ref_source\20s\ref_sq_bs_th2_1.mat", "th2");
% save("ref_dth1.mat", "dth1");
% save("ref_dth2.mat", "dth2");