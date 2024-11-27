% converting mat references into 2 seconds

drawtime = 2;

% Specify the source and destination folders
sourceFolder = 'C:\Users\Sez26\Documents\Arduino\MVNLC\control\Uni2024_MVNLC\Simulink_Simulation\ref_source\20s'; % Replace with the source folder path
destinationFolder = 'C:\Users\Sez26\Documents\Arduino\MVNLC\control\Uni2024_MVNLC\Simulink_Simulation\ref_source\2s'; % Replace with the destination folder path

% Create the destination folder if it does not exist
if ~exist(destinationFolder, 'dir')
    mkdir(destinationFolder);
end

% List all .mat files in the source folder
matFiles = dir(fullfile(sourceFolder, '*.mat'));

% Specify the row to modify (e.g., 2nd row)
rowToModify = 1;

% Loop through each .mat file
for k = 1:length(matFiles)
    % Load the .mat file
    sourceFile = fullfile(sourceFolder, matFiles(k).name);
    data = load(sourceFile);
    variableNames = fieldnames(data);
    varData = data.(variableNames{1});
    varData(rowToModify, :) = linspace(0,drawtime,length(varData));
    % % Modify the specified row in all variables
    % variableNames = fieldnames(data);
    % for i = 1:numel(variableNames)
    %     varData = data.(variableNames{i});
    % 
    %     if ismatrix(varData) && size(varData, 1) >= rowToModify
    %         % Apply the modification operation to the specified row
    % 
    %         data.(variableNames{i}) = varData;
    %     end
    % end
    % Save the modified data into the destination folder
    destinationFile = fullfile(destinationFolder, matFiles(k).name);
    save(destinationFile, 'varData');
end

disp('Processing completed.');
