function plot_response(data, L1, L2, start, color)
    %plot response
    [xData, yData] = get_Data_from(data, L1, L2,"thetas");
    % Initial point
    % plot(xData_1(1), yData_1(1), 'o', 'MarkerSize', 5,'MarkerFaceColor', [0, 0.4470, 0.7410]); 
    plot(xData(start:end), yData(start:end),string(color),'LineWidth', 1.5);
    disp(xData(start))
    disp(yData(start))
end

function plot_reference(data, L1, L2, start, color)
    %plot references
    [xData, yData] = get_Data_from(data, L1, L2,"response");
    plot(xData(start:end), yData(start:end),string(color),'LineWidth', 1.5);
    xlim([min(xData)-25, max(xData)+25]);
    ylim([min(yData)-25, max(yData)+25]);
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

function plot_manual(color)
    x_start = 42;
    y_start = -137;
    
    % Define the side length of the square
    side_length = 86;
    
    % Calculate the coordinates of the square's corners
    x = [x_start, x_start + side_length, x_start + side_length, x_start, x_start];
    y = [y_start, y_start, y_start + side_length, y_start + side_length, y_start];
    plot(x, y, string(color), 'LineWidth',1.5);
    xlim([min(x)-25, max(x)+25]);
    ylim([min(y)-25, max(y)+25]);
end

% Create figure
clear figure;
figure('Renderer', 'painters', 'Position', [10 10 550 500]);
hold on;

start = 500; 
plot_image('triangle.jpg');
%plot_manual('k');
plot_reference(PID_triangle_backlash, L1, L2, start, 'k');
plot_response(PID_triangle_backlash, L1, L2, start, 'r');

%Graph settings ----------------------------
legend('Real Response','Target shape','PID Model Response');
xlabel('X (mm)');
ylabel('Y (mm)');
grid on; axis equal;
set(gca,'fontsize', 14) 


