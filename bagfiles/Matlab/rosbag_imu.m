%% Reading data from Rosbag for plotting (imu Topic)
clc, clear all, close all

% Get the rosbag in question 
bag        = rosbag('2019-01-07-17-38-12.bag'); % Import file name
bSel       = select(bag,'Topic','/imu');
msgStructs = readMessages(bSel,'DataFormat','struct');

%% Setup
Firstmsg     = msgStructs{1};         % Get the first struct for analysis
InitialTime  = Firstmsg.Header.Seq;   % Subtracted from each subsequent frame
sizeOfStruct = length(msgStructs);
MyValues_imu = zeros(3,sizeOfStruct); % Initialize matrix
timeStamp    = zeros(1,sizeOfStruct); % Initialize vector 

%% General formula for getting relevant data from struct
for i = 1:sizeOfStruct
    StructTemp        = msgStructs{i};
    timeStamp(i)      = StructTemp.Header.Seq - InitialTime; % General formula for getting a proper time vector
    MyValues_imu(1,i) = StructTemp.AngularVelocity.X;
    MyValues_imu(2,i) = StructTemp.AngularVelocity.Y; 
    MyValues_imu(3,i) = StructTemp.AngularVelocity.Z;
end

%% Curve fitting
% Aquire these from graph by data cursor tool
% x1 = ; 
% x2 = ;

% Extract values between these points
% A   = MyValues_imu(1,x1:x2);
% B   = timeStamp(x1:x2);
% l   = length(B);
% p   = polyfit(B,A,1); % Get coefficients for y = kx + m, p = [k m]
% A_p = polyval(p,B);   % Evaluates the polynomial over vector B
% 
% figure(2)
% subplot(2,1,1)
% plot(B,A,'-o')
% xlabel('Time')
% ylabel('Angular Veclocity')
% legend('Data from imu')
% title('Real flight')
% 
% subplot(2,1,2)
% plot(B,A_p,'-o')
% xlabel('Time')
% ylabel('Angular Veclocity')
% legend('Interpolated data')
% title('Straight line approximation')
% 
% Control_Constant_Roll = p(1)*100 % 100Hz sampling

%% Plotting
figure(1)
plot(timeStamp,MyValues_imu(1,:))
xlabel('Time in 10s of ms')
ylabel('Angular Veclocity')
legend('')
title('Drone performing small oscillations around #-axis')

% subplot(4,1,4)
% plot(timeStamp,MyValues_imu(1,:),B,A_p,'-o')
% xlabel('Time in 10s of ms')
% ylabel('Angular Velocity')
% legend('Data from imu','Interpolated data')
% title('Aquiring constant from test flight (#)')