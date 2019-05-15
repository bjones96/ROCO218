%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% all rights reserved
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 10/02/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculate non-linear Arduino suitable simulation of uncontrolled inverted pendulum

close all
clear all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% consider equation
% d2theta/dt2 - a1*dtheta/dt -a2*theta + b0* ndu/dt + b1*u
% this equation captures the dynamics of the inverted pendulum 
% values are either given or calculated via equation
l = 0.64/2;
m = 0.314;
u = 0.05;
g = 9.81;
I = ((1/12) * m*(l^2));
theta = 0.01;

b0 =  (m*l)/(I+m*l^2);
b1 = 0;
a0 = 1;
a1 = u/(I+m*l^2);
a2 = -(m*g*l)/(I+m*l^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup time points
dt =  0.0400;
Tfinal = 5;
t = 0:dt:Tfinal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build linearized state space matrices from differential equation
% for velocity control
% % get inverted configuation
%Values used to calculate K
%A_inverted = [0 1;-a2 -a1];
A = [0 1 0 0;-a2 -a1 0 0;0 0 0 0;0 0 1 0;];     
B = [b0;-a1*b0;1;0];     
C = [1 0 0 0;];
D = 0;
%Values used for Luenberger calculation
Al = [0 1;-a2 -a1];
Cl = [1 0];
%K = [];     

%Luenberger observer
P = 10*[-1 -1.1];
L = place(Al, Cl', P);
disp('Values for L');               %Values displayed in the command window
disp(L);

% observer gain - use more aggressive poles than for controller
Px = 8*[-1 -1.1 -0.5 -0.6];
K = place(A, B, Px);
disp('Values for K');               %Values displayed in the command window
disp(K);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
titleMessage = 'Inverted Pendulum: ID 10540384';
disp(titleMessage)

% initialize arrays
xData = [];
yData=[];
tData=[];
kickFlag=[];

% for sub-loop runs
runs = 2;
for kick=1:runs
        
    % for each run randomly perturb intial condition
    x0 = [0; 1 * (rand - 0.5);0;0];
  
    % run Euler integration
    [y, t, x] = StateSpaceIntegrator(@CBNLVCPend, a1, a2, b0,  C, D, K, t, x0, L);
    
    % get time
    newTime = (kick-1) * t(end) + t;
    
    % just show kick arrow for short time after kick
    frames = length(t);
    kickFlagK = zeros(1,frames);
    if(x0(2) > 0)
        % scale arrow to size of kick
        kickFlagK(1: floor(frames/4)) = -abs(x0(2));
    else
        % scale arrow to size of kick
        kickFlagK(1: floor(frames/4)) = abs(x0(2));
    end
    
    % concatenate data between runs
    tData = [tData newTime];
    yData = [yData y];
    xData = [xData x];
    kickFlag = [kickFlag kickFlagK];
end

% plot out the state variables
PlotStateVariable2x2(xData, tData, titleMessage);

% for all time point animate the results
figure
range=1;

% cart not moving so set distance to zero
distance = zeros( size(yData));

% use animate function
AnimatePendulumCart( (yData + pi),  distance, 0.6, tData, range, kickFlag, titleMessage);


