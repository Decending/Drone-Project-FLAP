%% Reading data from Rosbag for plotting (Vicon topic)
clc, close all, clear all

% Get the rosbag in question 
bag        = rosbag('2019-01-07-17-38-12.bag'); % Import file name
bSel       = select(bag,'Topic','/vicon/flap/flap/odom');
msgStructs = readMessages(bSel,'DataFormat','struct');

%% Setup
Firstmsg      = msgStructs{1}; % Get the first struct for analysis
InitialTime   = Firstmsg.Header.Seq; % Timestamp in first frame needs to be subtracted from subsequent frame
sizeOfStruct  = length(msgStructs);
MyValues_odom = zeros(6,sizeOfStruct); % Initialize matrix
timeStamp     = zeros(1,sizeOfStruct); % Initialize vector 

%% General formula for getting relevant data from struct
for i = 1:sizeOfStruct
    StructTemp         = msgStructs{i};
    timeStamp(i)       = StructTemp.Header.Seq - InitialTime; % General formula for getting a proper time vector
    MyValues_odom(1,i) = StructTemp.Pose.Pose.Position.X;
    MyValues_odom(2,i) = StructTemp.Pose.Pose.Position.Y;
    MyValues_odom(3,i) = StructTemp.Pose.Pose.Position.Z;
    MyValues_odom(4,i) = StructTemp.Twist.Twist.Linear.X;
    MyValues_odom(5,i) = StructTemp.Twist.Twist.Linear.Y;
    MyValues_odom(6,i) = StructTemp.Twist.Twist.Linear.Z;
end

%% Plotting
% Position
figure(1)
subplot(3,1,1)
plot(timeStamp,MyValues_odom(1,:))
xlabel('Time')
ylabel('Position')
legend('X')
title('Position coordinate from Vicon (Test#)')

subplot(3,1,2)
plot(timeStamp,MyValues_odom(2,:))
xlabel('Time')
ylabel('Position')
legend('Y')

subplot(3,1,3)
plot(timeStamp,MyValues_odom(3,:))
xlabel('Time')
ylabel('Position')
legend('Z')

% Velocity
figure(2)
subplot(3,1,1)
plot(timeStamp,MyValues_odom(4,:))
xlabel('Time')
ylabel('Angular Veclocity')
legend('X')
title('Angular velocity from Vicon (Test#)')

subplot(3,1,2)
plot(timeStamp,MyValues_odom(5,:))
xlabel('Time')
ylabel('Angular Veclocity')
legend('Y')

subplot(3,1,3)
plot(timeStamp,MyValues_odom(6,:))
xlabel('Time')
ylabel('Angular Veclocity')
legend('Z')