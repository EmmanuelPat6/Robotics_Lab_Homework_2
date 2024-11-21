unzip("file.zip_name");
folderPath = fullfile(pwd,"folder_name");
bagReader = ros2bagreader(folderPath);
baginfo = ros2("bag","info",folderPath)
msgs = readMessages(bagReader);

efforts = [];
for i = 1:length(msgs)
efforts = [efforts; msgs{i}.data'];
end

t_start = 0;       % Initial Time for Plotting
num_points = 674;  % Number of Values (for this look at the number of the rows of the created variable 'efforts')
t_end = num_points*0.02;        % Final Time for Plotting (20 ms of the publisher

% Time Array
time_vector = linspace(t_start, t_end, num_points);


% Timeseries Object
efforts_ts = timeseries(efforts, time_vector);

% Plotting
figure;
plot(efforts_ts.Time, efforts_ts.Data);
xlabel('Time (s)');
ylabel('Nm');
title('Efforts');
grid on;
