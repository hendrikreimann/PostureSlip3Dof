% function eta = lorentzNoise(numberOfSamples, timeStep, alpha, c)
% 
% Hendrik Reimann, 2012
% Institut fuer Neuroinformatik, Ruhr-Universitaet Bochum, Germany
% adapted for use in posture model simulation

function [eta, xi] = lorentzNoise(stream, time, timeStep, alpha, c)

time = timeStep:timeStep:time;
xi = c * 1 / sqrt(timeStep) * randn(stream, 1, length(time));
eta = zeros(1, length(time));

for i_time = 2:length(time)
    etaDot = - alpha * eta(i_time-1) + xi(i_time-1);
    eta(i_time) = eta(i_time-1) + timeStep*etaDot;
end


return;