% function calculating the velocity profile resulting in a ramp displacement over a given time and distance

function velocity = sinusoidalDisplacementVelocity(currentTime, rampStart, rampEnd, displacementDistance)
% to achieve a real ramp, the acceleration would have to be infinite
% function approximates the jump in velocity with a sigmoid function instead


    rampTime = rampEnd - rampStart;
    peakVelocity = 2*displacementDistance / rampTime;

    
    velocity = peakVelocity*(1-cos((currentTime-rampStart)*(rampEnd-rampStart)^(-1)*2*pi))*0.5;
    velocity(currentTime<rampStart) = 0;
    velocity(currentTime>rampEnd) = 0;
end