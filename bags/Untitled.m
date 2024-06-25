bagFile = 'test.bag';

% Load the bag file
bag = rosbag(bagFile);

% Select the topics
xTopic = select(bag, 'Topic', '/unnamed_robot/x');
yTopic = select(bag, 'Topic', '/y');
rx0Topic = select(bag, 'Topic', '/rx0');
ry0Topic = select(bag, 'Topic', '/ry0');

% Read the messages
xMsgs = readMessages(xTopic, 'DataFormat', 'struct');
yMsgs = readMessages(yTopic, 'DataFormat', 'struct');
rx0Msgs = readMessages(rx0Topic, 'DataFormat', 'struct');
ry0Msgs = readMessages(ry0Topic, 'DataFormat', 'struct');

% Extract the data and timestamps
xData = cellfun(@(m) m.Data, xMsgs);
yData = cellfun(@(m) m.Data, yMsgs);
rx0Data = cellfun(@(m) m.Data, rx0Msgs);
ry0Data = cellfun(@(m) m.Data, ry0Msgs);

xTimes = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)/1e9, xMsgs);
yTimes = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)/1e9, yMsgs);
rx0Times = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)/1e9, rx0Msgs);
ry0Times = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)/1e9, ry0Msgs);

% Plot x and y data
figure;
plot(xTimes, xData, 'DisplayName', 'x');
hold on;
plot(yTimes, yData, 'DisplayName', 'y');
xlabel('Time [s]');
ylabel('Value');
legend;
title('x and y data over time');
hold off;

% Plot rx0 and ry0 data
figure;
plot(rx0Times, rx0Data, 'DisplayName', 'rx0');
hold on;
plot(ry0Times, ry0Data, 'DisplayName', 'ry0');
xlabel('Time [s]');
ylabel('Value');
legend;
title('rx0 and ry0 data over time');
hold off;