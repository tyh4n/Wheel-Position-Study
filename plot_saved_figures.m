% Create a new figure with two subplots stacked vertically
figure;

% First subplot (center.fig)
subplot(2,1,1);  % Create the first subplot
h1 = openfig('center.fig', 'invisible');  % Open the first figure invisibly
axes1 = get(h1, 'CurrentAxes');  % Get the axes of the opened figure
new_axes1 = copyobj(axes1, gcf);  % Copy the entire axes to the new figure
set(new_axes1, 'Position', get(gca, 'Position'));  % Adjust the position to match subplot layout
xlabel('\alpha (rad)');  % Set x label
ylabel('\beta (rad)');  % Set y label
title('Center Plot');  % Set the title

% Second subplot (mirror.fig)
subplot(2,1,2);  % Create the second subplot
h2 = openfig('mirror.fig', 'invisible');  % Open the second figure invisibly
axes2 = get(h2, 'CurrentAxes');  % Get the axes of the second figure
new_axes2 = copyobj(axes2, gcf);  % Copy the entire axes to the new figure
set(new_axes2, 'Position', get(gca, 'Position'));  % Adjust the position to match subplot layout
xlabel('\alpha (rad)');  % Set x label
ylabel('\beta (rad)');  % Set y label
title('Mirror Plot');  % Set the title

% Close the original figures
close(h1);
close(h2);
