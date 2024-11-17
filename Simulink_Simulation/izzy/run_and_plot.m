open_system('Plant_1');

% Enable logging of the output signal in the Simulink model
set_param('Plant_1', 'SimulationMode', 'normal', 'Solver', 'ode45');
set_param('Plant_1', 'SaveOutput', 'on');

for i = 1:5
    % Change the 'Gain' parameter of the block
    new_gain_value = i * 10;  % Example: changing gain from 10 to 50
    set_param('Plant_1/PID Controller', 'Gain', num2str(new_gain_value));
    
    % Run the simulation
    sim('Plant_1');
    
    % Optionally, extract or process results
    % Example: If your model has an output, you can get the simulation result
    output = sim('Plant_1');  % or use `logsout` if you've logged signals
    disp(['Simulation ' num2str(i) ' with Gain = ' num2str(new_gain_value)]);

    % Run simulation
    simOut = sim('my_model');
    
    % Access the simulation results (e.g., a signal logged during the simulation)
    output_signal = simOut.get('yout'); % Replace 'yout' with the appropriate signal name
    disp(['Simulation ' num2str(i) ' with Gain = ' num2str(new_gain_value)]);
end