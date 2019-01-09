%% Reading data from Rosbag for plotting (motor_speed Topic)
clc, clear all, close all

% Get the rosbag in question 
bag        = rosbag('2019-01-04-16-46-33.bag'); % Import file name
bSel       = select(bag,'Topic','/command/motor_speed');
msgStructs = readMessages(bSel,'DataFormat','struct');

%% Setup
Firstmsg     = msgStructs{1}; % Get the first struct for analysis
InitialTime  = Firstmsg.Header.Seq; % Subtracted from each subsequent frame
sizeOfStruct = length(msgStructs);
MyValues_ms  = zeros(4,sizeOfStruct); % Initialize matrix
timeStamp    = zeros(1,sizeOfStruct); % Initialize vector 

%% General formula for getting relevant data from struct
for i = 1:sizeOfStruct
    StructTemp       = msgStructs{i};
    timeStamp(i)     = StructTemp.Header.Seq - InitialTime; % General formula for getting a proper time vector
    MyValues_ms(1,i) = StructTemp.Normalized(1); % propeller1
    MyValues_ms(2,i) = StructTemp.Normalized(2); % porpeller2
    MyValues_ms(3,i) = StructTemp.Normalized(5); % flap1
    MyValues_ms(4,i) = StructTemp.Normalized(6); % flap2
end

uavg = (max(MyValues_ms(1,:)) + max(MyValues_ms(2,:)))/2;
% Plotting
figure(1)
subplot(4,1,1)
plot(timeStamp,MyValues_ms(1,:))
xlabel('Time')
ylabel('Control signal')
legend('')
title('Control input data for Propeller 1 ')

subplot(4,1,2)
plot(timeStamp,MyValues_ms(2,:))
xlabel('Time')
ylabel('Control signal')
legend('')
title('Control input data for Propeller 2')

subplot(4,1,3)
plot(timeStamp,MyValues_ms(3,:))
xlabel('Time')
ylabel('Control signal')
legend('Servo motor 1 voltage output')
title('Control input data for Servo motor 1')

subplot(4,1,4)
plot(timeStamp,MyValues_ms(4,:))
xlabel('Time')
ylabel('Control signal')
legend('')
title('Control input data for Servo motor 2')