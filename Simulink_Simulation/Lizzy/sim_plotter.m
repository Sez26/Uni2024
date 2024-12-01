function plot_response(data, L1, L2, start, color)
    %plot response
    [xData, yData] = get_Data_from(data, L1, L2,"thetas");
    % Initial point
    % plot(xData_1(1), yData_1(1), 'o', 'MarkerSize', 5,'MarkerFaceColor', [0, 0.4470, 0.7410]); 
    plot(xData(start:end), yData(start:end),string(color),'LineWidth', 1.5);
end

function plot_reference(data, L1, L2, start, color)
    %plot references
    [xData, yData] = get_Data_from(data, L1, L2,"response");
    plot(xData(start:end), yData(start:end),string(color),'LineWidth', 1.5);
    %xlim([min(xData)-15, max(xData)+15]);
    %ylim([min(yData)-15, max(yData)+15]);
end

function plot_image(image)
    img = imread(string(image));
    img = (flipdim(img,1));
    grayImg = rgb2gray(img);
    threshold = 50;  % Adjust if needed
    blackPixels = grayImg < threshold;  % Logical mask of black pixels
    [row, col] = find(blackPixels);
    
    %transformations
    y_transform = -170;
    x_transform = 30;
    row = (row*0.14) + y_transform; 
    col = (col*0.14) + x_transform;
    scatter(col, row, 0.5, 'MarkerFaceColor', '#D3D3D3', ...  % Grey color
            'MarkerEdgeColor', '#D3D3D3', 'MarkerFaceAlpha', 0);  % Set opacity

end

% Create figure
clear figure;
figure('Renderer', 'painters', 'Position', [10 10 550 500]);
hold on;

start = 500; 
plot_image('square1.jpg');
plot_reference(PID_square_backlash, L1, L2, start, 'k'); 
plot_response(PID_square_backlash, L1, L2, start, 'r');

%Graph settings ----------------------------
legend('Target shape','No backlash','Backlash modelled'); %'Linear model','Nonlinear model','State feedback control','backlash','Target shape');
xlabel('X (mm)');
ylabel('Y (mm)');
grid on; axis equal;
set(gca,'fontsize', 14) 


