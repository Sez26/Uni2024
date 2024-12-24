%% import reference signals
% converting array into timesseries type
addpath('ref_sig\')

filename = 'ref_tri_bs_2.mat';
data = load(filename);
time_arr = data.reference(:,1)';
% ts3 = timeseries((1:5)',[0 10 20 30 40]);
% ref_time = timeseries(data.thetas, time_arr);
th1 = [time_arr; data.reference(:,2)'];
th2 = [time_arr; data.reference(:,3)'];

% dth1 = [time_arr; data.reference(:,4)'];
% dth2 = [time_arr; data.reference(:,5)'];

save(".\ref_source\20s\ref_tri_bs_th1_2.mat", "th1");
save(".\ref_source\20s\ref_tri_bs_th2_2.mat", "th2");
% save("ref_dth1.mat", "dth1");
% save("ref_dth2.mat", "dth2");