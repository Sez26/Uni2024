function plot_bode_magnitude(sys, x_label, y_label)
    % Input:
    % sys: 2x2 linear system object (state-space or transfer function)
    % x_label: custom label for the x-axis
    % y_label: custom label for the y-axis

    % Create a Bode plot
    [mag, freq] = bode(sys);

    % mag is a 4D array: [rows, cols, 1, frequency points]
    % Extract the magnitude data for each element
    mag = squeeze(mag); % Reduce dimensionality

    % Set up the frequency range (rad/sec)
    freq = squeeze(freq); % Frequency is a column vector

    % Plot the magnitude response
    figure;
    hold on;
    for i = 1:size(mag, 1) % Loop over rows of the system
        for j = 1:size(mag, 2) % Loop over columns of the system
            plot(freq, 20*log10(mag(i, j, :)), 'DisplayName', sprintf('Element (%d,%d)', i, j));
        end
    end
    hold off;

    % Customizing the plot
    xlabel(x_label); % Custom x-axis label
    ylabel(y_label); % Custom y-axis label
    title('Bode Magnitude Plot');
    grid on;
    legend show; % Display the legend
end


% plot_bode_magnitude(linsys6,"test1","test2")