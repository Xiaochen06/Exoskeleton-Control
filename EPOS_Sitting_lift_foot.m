clc
clear

%% motor setup
Motor1 = Epos4(1,0);
% function res = Epos4(NodeID, usbID)
% NodeID and usdID are set in the EPOS Studio
disp('Motor1')
Motor1.SetOperationMode(OperationModes.ProfilePositionMode);
Motor1.EnableNode;
Motor1.ClearErrorState;

Motor2 = Epos4(2,1);
disp('Motor2')
Motor2.SetOperationMode(OperationModes.ProfilePositionMode);
Motor2.EnableNode;
Motor2.ClearErrorState;
%% neutral position
% the subject standing with tight cables
Motor1.MotionInPosition(50000);
% Motor2.MotionInPosition(50000);

%% neutral state position
Pos10 = Motor1.ActualPosition
Pos20 = Motor2.ActualPosition

%% 
delete(Motor1);
delete(Motor2);
%% motor setup again
Motor1 = Epos4(1,0);
% function res = Epos4(NodeID, usbID)
% NodeID and usdID are set in the EPOS Studio
disp('Motor1')
Motor1.SetOperationMode(OperationModes.ProfilePositionMode);
Motor1.EnableNode;
Motor1.ClearErrorState;

Motor2 = Epos4(2,1);
disp('Motor2')
Motor2.SetOperationMode(OperationModes.ProfilePositionMode);
Motor2.EnableNode;
Motor2.ClearErrorState;

%% position control parameter calculation
% 

% parameter of the encoder
% counts per turn: 1024
% state changes in one round: 1024*4*(6877/56) inc
% radius of the pulley: 14 mm

% l_calf = 200; % mm
% l_shoe = 180;
% h_shoe = 90;
% r_pulley = 14;
% 
% angle_neutral = asin(h_shoe/l_shoe)/pi*180 + 90; % unit: degree
% angle_N = angle_neutral*pi/180; %unit: 
% cable_neutral = sqrt(l_calf^2+l_shoe^2-2*l_calf*l_shoe*cos(angle_N)); % cable length
% MA_neutral = (l_calf*l_shoe*sin(angle_N))/sqrt(l_calf^2+l_shoe^2-2*l_calf*l_shoe*cos(angle_N)); % moment arm
%  
% angle_plantar = 15; % 15 degree plantar flexion
% angle_initial = angle_neutral + angle_plantar; 
% angle_I = angle_initial*pi/180;
% cable_initial = sqrt(l_calf^2+l_shoe^2-2*l_calf*l_shoe*cos(angle_I));
% MA_initial = (l_calf*l_shoe*sin(angle_I))/sqrt(l_calf^2+l_shoe^2-2*l_calf*l_shoe*cos(angle_I));
% 
% cable_change = cable_initial - cable_neutral;
% position_change = cable_change/(2*pi*r_pulley)*1024*4*(6877/56);

%% trajetory 
Motor1.SetOperationMode(OperationModes.ProfilePositionMode);
Motor1.EnableNode;
Motor1.ClearErrorState;
Motor2.SetOperationMode(OperationModes.ProfilePositionMode);
Motor2.EnableNode;
Motor2.ClearErrorState;
% the cable position in the plantar posture
Pos11 = Motor1.ActualPosition;% absolute position
Pos21 = Motor2.ActualPosition;

% Pos10 = Motor1.ActualPosition + 125000;
% Pos20 = Motor2.ActualPosition + 150000;

% the desired length that the cable should be shortened to lift the foot to the neutral position
Position_change1 = Pos10 - Pos11; % relative position [inc]
Position_change2 = Pos20 - Pos21;

t = 0.75; % define the time of lifting the foot

v_max1 = Position_change1/t*2/(1024*4)*60; % the maximum speed [inc/s -> rpm]
v_max2 = Position_change2/t*2/(1024*4)*60; % the maximum speed [inc/s -> rpm]
if v_max1 > 8000
   v_max1 = 8000; 
end
if v_max2 > 8000
   v_max2 = 8000; 
end
a1 = v_max1/t*2; % the acceleration and deceleration
a2 = v_max2/t*2; % the acceleration and deceleration

sn = 70; % define the sampling number
frameCount = 1;
RawPosition = zeros(sn,4); 
RawVelocity = zeros(sn,2); 
RawCurrent = zeros(sn,2); 
tic
Motor1.MotionInPosition(Position_change1,v_max1,a1);
Motor2.MotionInPosition(Position_change2,v_max1,a2);

% record the data
while(frameCount <= sn)
    
    RawPosition(frameCount,1) = Motor1.ActualPosition - Pos11;
    RawPosition(frameCount,2) = Motor2.ActualPosition - Pos21;
    
    RawVelocity(frameCount,1) = Motor1.ActualVelocity;
    RawVelocity(frameCount,2) = Motor2.ActualVelocity;
    
    RawCurrent(frameCount,1) = Motor1.ActualCurrent; 
    RawCurrent(frameCount,2) = Motor2.ActualCurrent;  
    
    frameCount = frameCount +1;    
end

toc

figure;
plot(RawPosition(:,1));
hold on
plot(RawPosition(:,2));
hold off

figure;
plot(RawVelocity(:,1));
hold on
plot(RawVelocity(:,2));
hold off

figure;
plot(RawCurrent(:,1));
hold on
plot(RawCurrent(:,2));
hold off

%% 
delete(Motor1);
delete(Motor2);
