%torque graphs
% get e and t from timeseries data
function [err, time] = get_Data(workspace_name, L1, L2, start)
    if nonlinear
        th1Data = workspace_name.get('th1').Values.Data(:,1);
        th2Data = workspace_name.get('th1').Values.Data(:,2);
    else
        th1Data = workspace_name.get('th1').Values.Data;
        th2Data = workspace_name.get('th2').Values.Data;
    end

    %Return time
    time = workspace_name.get('ref_th1').Values.Time; %verified both time series are the same
    time = time(start:end);

    %Calculate error distances
    response = [xData_th1(start:end), yData_th2(start:end)];
    ref = [xData_ref1(start:end), yData_ref2(start:end)];
    err = sqrt(sum((response - ref).^2, 2));
end