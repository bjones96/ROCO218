function [y, tout, xout] = StateSpaceIntegrator(CBNLVCPend, a1, a2, b0,  C, D, K, t, x0, L)
% non-linear model state space feedback velocity control using C-language compatible formulation
% performs integration by Euler's method
% VCPendDotCB is the callback function to compute xDot 
% a1, a2, b0 are coedfficients  
% C, D ame the state space model output matrices
% K id the state feedback control ggain - unused here
% t is a vector of time samples
% x0 is the initial state vector
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

% get signal length
len = length(t) -1;

%L;

% init output
y = zeros(1,len);
xout = zeros(4,len);

% record the initial state
xout(:, 1) = x0;
x = x0;

% for all remaining data points, simulate state-space model using C-language compatible formulation
for idx = 1:len
    
    % controlled input for loop
    u = -K*x;
    
    % record time
    tout(idx) = t(idx);

    % get the duration between updates
    h = t(idx+1) - t(idx);
    
    % calculate state derivative from non-linear pendulum equations
    xDot = CBNLVCPend(a1, a2, b0, x, u);
    
    % update the state using Euler integration
    x(1) = x(1) + h * xDot(1);
    x(2) = x(2) + h * xDot(2);
    x(3) = x(3) + h * xDot(3);
    x(4) = x(4) + h * xDot(4);      
    
    % record the state
    xout(:, idx) = x;
    
    %correction term
    %%ycorr = L * (y - C * xhat);
    
    %update observer state xhat using euler integration
    %%xhat = xhat + h * (A * xhat + B * U + ycorr);
    
    % calculate output from theta and thetaDot state
    y(idx)= C(1) * x(1) + C(2) * x(2) + D(1) * u;
end



