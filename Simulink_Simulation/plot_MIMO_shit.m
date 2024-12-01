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

% Loop through the first 8 .mat files (or fewer if less than 8 exist)
maxFiles = min(8, length(fileList));
for i = 1:maxFiles
    % Load the .mat file
    data = load(fullfile(folderPath, fileList(i).name));
    
    % Assume the time-series variable is the only variable in the .mat file
    vars = fieldnames(data);
    tsData = data.(vars{1});
    
    % Check if the variable is a timeseries object
    if ~isa(tsData, 'timeseries')
        warning('File %s does not contain a timeseries object. Skipping.', fileList(i).name);
        continue;
    end
    
    % Extract the data from the timeseries object
    signal = tsData.Data;

    % find index when time is 2 seconds
    idx_start = find(data.data.Time>=2);
    idx_start = idx_start(1);
    
    % Extract the latter half of the signal
    latterHalf = signal(idx_start:end)-signal(idx_start);
    time = data.data.Time(idx_start:end)-data.data.Time(idx_start);
    
    % Plot on a semilogarithmic scale
    % semilogy(data.data.Time(floor(signalLength/2):end)-2, latterHalf, 'DisplayName', fileList(i).name);
    semilogy(time(5:end), latterHalf(5:end), LineWidth=1.5);
end

% Add legend to the plot
legend(["OFB control \theta_1", "OFB control \theta_2", "SFB control \theta_1", "SFB control \theta_2", "SFBIA control \theta_1", "SFBIA control \theta_2", "PID SISO control \theta_1", "PID SISO control \theta_2"], Location="northwest");
set(gca, 'YScale', 'log')
hold off;