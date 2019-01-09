%% Reading data from Rosbag for plotting (rc Topic)
clc, close all, clear all

% Get the rosbag in question 
bag        = rosbag('2019-01-07-17-38-12.bag'); % Import file name
bSel       = select(bag,'Topic','/rc');
msgStructs = readMessages(bSel,'DataFormat','struct');

%% Setup
Firstmsg     = msgStructs{1};         % Get the first struct for analysis
InitialTime  = Firstmsg.Header.Seq;   % Subtracted from each subsequent frame
sizeOfStruct = length(msgStructs);
MyValues_rc  = zeros(1,sizeOfStruct); % Initialize matrix
timeStamp    = zeros(1,sizeOfStruct); % Initialize vector 

%% General formula for getting relevant data from struct
for i = 1:sizeOfStruct
    StructTemp       = msgStructs{i};
    timeStamp(i)     = StructTemp.Header.Seq - InitialTime; % General formula for getting a proper time vector
    MyValues_rc(1,i) = StructTemp.Axes(3);
end

%% Plotting
figure(1)
plot(timeStamp,MyValues_rc(1,:))
xlabel('Time')
ylabel('Thrust input')
legend('RC thrust input')
title('RC controlled flight (Test#)')