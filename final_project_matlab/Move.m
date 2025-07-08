open_system("elsayed.slx"); % Open the Simulink model "elsayed.slx"
data=sim("elsayed.slx"); % Run the simulation of the model and store the results in the variable data

% Define the lengths of the arms and other parameters
l1 = 2; % Length of the first arm
l2 = 2; % Length of the second arm
d1 = 2; % Offset along the Z-axis for the first part
d3 = 2; % Additional offset along the Z-axis
d4 = 2; % Additional offset along the Z-axis
ze=8; % Final value on the Z-axis
xx=[0 0]; % X coordinates for the fixed arm
yy=[0 0]; % Y coordinates for the fixed arm
zz=[0 ze]; % Z coordinates for the fixed arm

% Calculate the coordinates for the first arm
px=[0 l1 * cos(act_th1(1))]; % X coordinates for the first arm's endpoint
py=[0 l1 * sin(act_th1(1))]; % Y coordinates for the first arm's endpoint
pxx=[l1 * cos(act_th1(1)), l1 * cos(act_th1(1)) + l2 * cos(act_th1(1) + act_th2(1))]; % X coordinates for the second arm's endpoint
pyy=[l1 * sin(act_th1(1)), l1 * sin(act_th1(1)) + l2 * sin(act_th1(1) + act_th2(1))]; % Y coordinates for the second arm's endpoint

% Loop for the first animation (movement of arm along Z-axis)
for i=1:0.5:8
    pz=[i i]; % Z coordinates for the arms during the movement

    figure(1), clf, % Clear the figure window
    plot3(xx, yy, zz, 'Color', 'b', 'LineWidth', 2); % Plot the fixed arm in blue
    hold on
    grid on
    axis([0 4 -1.5 1.5 0 10]); % Set the axis limits
    plot3(px, py, pz, 'Color', 'r', 'LineWidth', 2); % Plot the first arm in red
    hold on
    plot3(pxx, pyy, pz, 'Color', 'y', 'LineWidth', 2); % Plot the second arm in yellow
    hold on

    pause(0.5) % Pause for half a second before the next iteration
end

% Loop for plotting the desired trajectory and actual movement
for i = 1:100:size(xd)
    figure(1), clf, % Clear the figure window
    plot3(xd, yd, zd, 'b') % Plot the desired trajectory in blue
    hold on
    axis([0 4 -1.5 1.5 0 10]); % Set the axis limits
    grid on

    % Plotting the arms at each step
    p1 = [0, l1 * cos(act_th1(i))]; % X coordinates for the first joint
    p2 = [0, l1 * sin(act_th1(i))]; % Y coordinates for the first joint
    p3_z = [d1 + act_d2(i) - d3 - d4, d1 + act_d2(i) - d3 - d4]; % Z coordinates for the first joint
    p4_z = [d1 + act_d2(i) - d3 - d4, d1 + act_d2(i) - d3 - d4]; % Z coordinates for the second joint

    plot3(xx, yy, zz, 'Color', 'b', 'LineWidth', 2); % Plot the fixed arm in blue
    line(p1, p2, p3_z, 'Color', 'r', 'LineWidth', 2); % Plot the first arm segment in red
    p3 = [l1 * cos(act_th1(i)), l1 * cos(act_th1(i)) + l2 * cos(act_th1(i) + act_th2(i))]; % X coordinates for the second arm segment
    p4 = [l1 * sin(act_th1(i)), l1 * sin(act_th1(i)) + l2 * sin(act_th1(i) + act_th2(i))]; % Y coordinates for the second arm segment

    line(p3, p4, p4_z, 'Color', 'y', 'LineWidth', 2); % Plot the second arm segment in yellow
end

% Loop for second animation (movement of arm back along Z-axis)
for i=8:-0.5:4
    pz=[i i]; % Z coordinates for the arms during the movement

    figure(1), clf, % Clear the figure window
    plot3(xx, yy, zz, 'Color', 'b', 'LineWidth', 2); % Plot the fixed arm in blue
    hold on
    grid on
    axis([0 5 -1.5 1.5 0 10]); % Set the axis limits
    plot3(px, py, pz, 'Color', 'r', 'LineWidth', 2); % Plot the first arm in red
    hold on
    plot3(pxx, pyy, pz, 'Color', 'y', 'LineWidth', 2); % Plot the second arm in yellow
    hold on

    pause(0.5) % Pause for half a second before the next iteration
end

% Loop for plotting another desired trajectory and actual movement
for i = 1:100:size(xd)
    figure(1), clf, % Clear the figure window
    plot3(xd, yd, zd / 2, 'g') % Plot the desired trajectory in green
    hold on
    axis([0 4 -1.5 1.5 0 10]); % Set the axis limits
    grid on

    % Plotting the arms at each step
    p1 = [0, l1 * cos(act_th1(i))]; % X coordinates for the first joint
    p2 = [0, l1 * sin(act_th1(i))]; % Y coordinates for the first joint
    p3_z = [4 , 4]; % Z coordinates for the first joint
    p4_z = [4 , 4]; % Z coordinates for the second joint

    plot3(xx, yy, zz, 'Color', 'b', 'LineWidth', 2); % Plot the fixed arm in blue
    line(p1, p2, p3_z, 'Color', 'r', 'LineWidth', 2); % Plot the first arm segment in red
    p3 = [l1 * cos(act_th1(i)), l1 * cos(act_th1(i)) + l2 * cos(act_th1(i) + act_th2(i))]; % X coordinates for the second arm segment
    p4 = [l1 * sin(act_th1(i)), l1 * sin(act_th1(i)) + l2 * sin(act_th1(i) + act_th2(i))]; % Y coordinates for the second arm segment

    line(p3, p4, p4_z, 'Color', 'y', 'LineWidth', 2); % Plot the second arm segment in yellow
end

% Loop for third animation (movement of arm back along Z-axis)
for i=4:-0.5:1
    pz=[i i]; % Z coordinates for the arms during the movement

    figure(1), clf, % Clear the figure window
    plot3(xx, yy, zz, 'Color', 'b', 'LineWidth', 2); % Plot the fixed arm in blue
    hold on
    grid on
    axis([0 5 -1.5 1.5 0 10]); % Set the axis limits
    plot3(px, py, pz, 'Color', 'r', 'LineWidth', 2); % Plot the first arm in red
    hold on
    plot3(pxx, pyy, pz, 'Color', 'y', 'LineWidth', 2); % Plot the second arm in yellow
    hold on

    pause(0.5) % Pause for half a second before the next iteration
end

hold off % Turn off the hold to stop adding to the current plot

% Plot the desired vs actual trajectories for x and y
subplot(3,1,1);
plot(xd, yd, 'b--', x, y, 'r--') % Desired and actual paths for x and y
hold on
xlabel('x-axis');
ylabel('y-axis');
legend('desired','actual'); % Legend for desired and actual paths
title('path') % Title for the path plot

% Plot the desired vs actual theta1 values over time
subplot(3,1,2);
plot(time, des_th1, 'r', 'LineWidth', 1) % Desired theta1
hold on
plot(time, act_th1, 'g--', 'LineWidth', 2) % Actual theta1
grid on
box on
xlabel('x-axis');
ylabel('y-axis');
legend('des-th1', 'act-th1'); % Legend for desired and actual theta1
title('THETA-1') % Title for theta1 plot

% Plot the desired vs actual theta2 values over time
subplot(3,1,3);
plot(time, des_th2, 'b', 'LineWidth', 1) % Desired theta2
hold on
plot(time, act_th2, 'g--', 'LineWidth', 2) % Actual theta2
grid on
box on
xlabel('x-axis');
ylabel('y-axis');
legend('des-th2', 'act-th2'); % Legend for desired and actual theta2
title('THETA-2') % Title for theta2 plot
