% plotting integrated error
% Specify the folder containing the .mat files
folderPath = '.\MIMO_int_e';

% Get a list of all .mat files in the folder
fileList = dir(fullfile(folderPath, '*.mat'));

% Initialize figure for plotting
figure;
hold on;
grid on;
xlabel('Time (s)');
ylabel('Integrated Error (deg)');
% title('Latter Half of Time-Series Signals on Semilogarithmic Plot');
xlim([0,2])

L = 0.095;

% Loop through the first 8 .mat files (or fewer if less than 8 exist)
maxFiles = min(8, length(fileList));
for i = 1:length(fileList)/2
    % Load the .mat file
    datath1 = load(fullfile(folderPath, fileList(i).name));
    datath2 = load(fullfile(folderPath, fileList(i+1).name));
    
    % Assume the time-series variable is the only variable in the .mat file
    varsth1 = fieldnames(datath1);
    tsDatath1 = datath1.(varsth1{1});
    varsth2 = fieldnames(datath1);
    tsDatath2 = datath2.(varsth2{1});

    % Check if the variable is a timeseries object
    % if ~isa(tsData, 'timeseries')
    %     warning('File %s does not contain a timeseries object. Skipping.', fileList(i).name);
    %     continue;
    % end
    % 
    % Extract the data from the timeseries object
    signalth1 = tsDatath1.Data;
    signalth2 = tsDatath2.Data;
    % find index when time is 2 seconds
    idx_start = find(datath1.data.Time>=2);
    idx_start = idx_start(1);
    
    % Extract the latter half of the signal
    half_th1 = signalth1(idx_start:end)-signalth1(idx_start);
    half_th2 = signalth2(idx_start:end)-signalth2(idx_start);
    err = sqrt((L.*cos(half_th1)+L.*cos(half_th2)).^2 + (L.*sin(half_th1)+L.*sin(half_th2)).^2);
    time = datath1.data.Time(idx_start:end)-datath1.data.Time(idx_start);
    
    % Plot on a semilogarithmic scale
    % semilogy(data.data.Time(floor(signalLength/2):end)-2, latterHalf, 'DisplayName', fileList(i).name);
    semilogy(time(5:end), err(5:end), LineWidth=1.5);
end

% Add legend to the plot
legend(["OFB control \theta_1", "OFB control \theta_2", "SFB control \theta_1", "SFB control \theta_2", "SFBIA control \theta_1", "SFBIA control \theta_2", "PID SISO control \theta_1", "PID SISO control \theta_2"], Location="northwest");
set(gca, 'YScale', 'log')
hold off;