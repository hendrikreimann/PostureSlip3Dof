% class for storing correlated noise

classdef PostureNoise
    properties
        totalTime;
        samplingFrequency;
        bodyDofs;
        L;
        
        % noise vectors
        motorNoise;
        neuralNoise;
        referenceNoise;
        
        proprioceptionNoisePosition;
        proprioceptionNoiseVelocity;
        proprioceptionNoiseAcceleration;
        
        visionNoisePosition;
        visionNoiseVelocity;
        visionNoiseAcceleration;
        
        orientationNoisePosition;
        orientationNoiseVelocity;
        orientationNoiseAcceleration;
        
        trackingErrorNoisePosition;
        trackingErrorNoiseVelocity;
        trackingErrorNoiseAcceleration;
        
        comSensorNoisePosition;
        comSensorNoiseVelocity;
        comSensorNoiseAcceleration;
        
        copSensorNoisePosition;
        torqueSensorNoise;
        
        headJacobianRepresentationNoise;
        comJacobianRepresentationNoise;
        inertiaRepresentationNoise;
        
        
    end
    methods
        function obj = PostureNoise(samplingFrequency, totalTime, bodyDofs)
            obj.samplingFrequency = samplingFrequency;
            obj.totalTime = totalTime;
            obj.bodyDofs = bodyDofs;
            obj.L = samplingFrequency * totalTime;
            
            obj.motorNoise = zeros(bodyDofs, obj.L);
            obj.neuralNoise = zeros(bodyDofs, obj.L);
            obj.referenceNoise = zeros(bodyDofs, obj.L);

            obj.proprioceptionNoisePosition = zeros(bodyDofs, obj.L);
            obj.proprioceptionNoiseVelocity = zeros(bodyDofs, obj.L);
            obj.proprioceptionNoiseAcceleration = zeros(bodyDofs, obj.L);

            obj.visionNoisePosition = zeros(1, obj.L);
            obj.visionNoiseVelocity = zeros(1, obj.L);
            obj.visionNoiseAcceleration = zeros(1, obj.L);

            obj.orientationNoisePosition = zeros(1, obj.L);
            obj.orientationNoiseVelocity = zeros(1, obj.L);
            obj.orientationNoiseAcceleration = zeros(1, obj.L);

            obj.trackingErrorNoisePosition = zeros(1, obj.L);
            obj.trackingErrorNoiseVelocity = zeros(1, obj.L);
            obj.trackingErrorNoiseAcceleration = zeros(1, obj.L);

            obj.comSensorNoisePosition = zeros(1, obj.L);
            obj.comSensorNoiseVelocity = zeros(1, obj.L);
            obj.comSensorNoiseAcceleration = zeros(1, obj.L);

            obj.copSensorNoisePosition = zeros(1, obj.L);
            obj.torqueSensorNoise = zeros(bodyDofs, obj.L);
            
            obj.headJacobianRepresentationNoise = zeros(bodyDofs, obj.L);
            obj.comJacobianRepresentationNoise = zeros(bodyDofs, obj.L);
            obj.inertiaRepresentationNoise = zeros(bodyDofs, bodyDofs, obj.L);
        end % constructor
        function obj = generateWhiteNoise(obj, parameters, stream)
            obj.bodyDofs = parameters.bodyDofs;
            
            obj.motorNoise = parameters.sigma_m * 1 / sqrt(parameters.eulerStep) * randn(stream, parameters.bodyDofs, obj.L);
            obj.neuralNoise = parameters.sigma_n * 1 / sqrt(parameters.eulerStep) * randn(stream, parameters.bodyDofs, obj.L);
            obj.referenceNoise = parameters.sigma_r * 1 / sqrt(parameters.eulerStep) * randn(stream, parameters.bodyDofs, obj.L);

            obj.proprioceptionNoisePosition = parameters.sigma_jp * 1 / sqrt(parameters.eulerStep) * randn(stream, parameters.bodyDofs, obj.L);
            obj.proprioceptionNoiseVelocity = parameters.sigma_jv * 1 / sqrt(parameters.eulerStep) * randn(stream, parameters.bodyDofs, obj.L);
            obj.proprioceptionNoiseAcceleration = parameters.sigma_ja * 1 / sqrt(parameters.eulerStep) * randn(stream, parameters.bodyDofs, obj.L);

            obj.visionNoisePosition = parameters.sigma_pp * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);
            obj.visionNoiseVelocity = parameters.sigma_pv * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);
            obj.visionNoiseAcceleration = parameters.sigma_pa * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);

            obj.orientationNoisePosition = parameters.sigma_op * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);
            obj.orientationNoiseVelocity = parameters.sigma_ov * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);
            obj.orientationNoiseAcceleration = parameters.sigma_oa * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);

            obj.trackingErrorNoisePosition = parameters.sigma_bp * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);
            obj.trackingErrorNoiseVelocity = parameters.sigma_bv * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);
            obj.trackingErrorNoiseAcceleration = parameters.sigma_ba * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);

            obj.comSensorNoisePosition = parameters.sigma_cp * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);
            obj.comSensorNoiseVelocity = parameters.sigma_cv * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);
            obj.comSensorNoiseAcceleration = parameters.sigma_ca * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);

            obj.copSensorNoisePosition = parameters.sigma_cop * 1 / sqrt(parameters.eulerStep) * randn(stream, 1, obj.L);
            obj.torqueSensorNoise = parameters.sigma_tp * 1 / sqrt(parameters.eulerStep) * randn(stream, parameters.bodyDofs, obj.L);
        end
%         function obj = generatePinkNoise(obj, parameters)
%             obj.bodyDofs = parameters.bodyDofs;
%             
%             for j = 1:obj.bodyDofs
%                 obj.motorNoise(j, :) = pinkNoise(stream, obj.L, 1, parameters.muscleNoisePower);
%                 obj.neuralNoise(j, :) = pinkNoise(stream, obj.L, 1, parameters.neuralNoisePower);
%                 obj.referenceNoise(j, :) = pinkNoise(stream, obj.L, 1, parameters.referenceNoisePower);
%                 obj.proprioceptionNoisePosition(j, :) = pinkNoise(stream, obj.L, 1, parameters.proprioceptionNoisePositionPower);
%                 obj.proprioceptionNoiseVelocity(j, :) = pinkNoise(stream, obj.L, 1, parameters.proprioceptionNoiseVelocityPower);
%                 obj.proprioceptionNoiseAcceleration(j, :) = pinkNoise(stream, obj.L, 1, parameters.proprioceptionNoiseAccelerationPower);
%             end
% 
%             obj.visionNoisePosition = pinkNoise(stream, obj.L, 1, parameters.visionNoisePositionPower);
%             obj.visionNoiseVelocity = pinkNoise(stream, obj.L, 1, parameters.visionNoiseVelocityPower);
%             obj.visionNoiseAcceleration = pinkNoise(stream, obj.L, 1, parameters.visionNoiseAccelerationPower);
% 
%             obj.orientationNoisePosition = pinkNoise(stream, obj.L, 1, parameters.orientationNoisePositionPower);
%             obj.orientationNoiseVelocity = pinkNoise(stream, obj.L, 1, parameters.orientationNoiseVelocityPower);
%             obj.orientationNoiseAcceleration = pinkNoise(stream, obj.L, 1, parameters.orientationNoiseAccelerationPower);
% 
%             obj.trackingErrorNoisePosition = pinkNoise(stream, obj.L, 1, parameters.trackingErrorNoisePositionPower);
%             obj.trackingErrorNoiseVelocity = pinkNoise(stream, obj.L, 1, parameters.trackingErrorNoiseVelocityPower);
%             obj.trackingErrorNoiseAcceleration = pinkNoise(stream, obj.L, 1, parameters.trackingErrorNoiseAccelerationPower);
% 
%             obj.comSensorNoisePosition = pinkNoise(stream, obj.L, 1, parameters.comSensorNoisePositionPower);
%             obj.comSensorNoiseVelocity = pinkNoise(stream, obj.L, 1, parameters.comSensorNoiseVelocityPower);
%             obj.comSensorNoiseAcceleration = pinkNoise(stream, obj.L, 1, parameters.comSensorNoiseAccelerationPower);
% 
%             obj.copSensorNoisePosition = pinkNoise(stream, obj.L, 1, parameters.copSensorNoisePositionPower);
%             obj.torqueSensorNoise = pinkNoise(stream, obj.L, 1, parameters.torqueSensorNoisePower);
%         end
        function obj = generateColoredNoise(obj, parameters, stream)
            timeStep = 1/obj.samplingFrequency;
            obj.bodyDofs = parameters.bodyDofs;
            
            for j = 1:obj.bodyDofs
                obj.motorNoise(j, :) = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_m);
                obj.neuralNoise(j, :) = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_n);
                obj.referenceNoise(j, :) = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_r);
                obj.proprioceptionNoisePosition(j, :) = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_jp);
                obj.proprioceptionNoiseVelocity(j, :) = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_jv);
                obj.proprioceptionNoiseAcceleration(j, :) = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_ja);
                obj.headJacobianRepresentationNoise(j, :) = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_jacv);
                obj.comJacobianRepresentationNoise(j, :) = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_jacm);
                for k = 1:obj.bodyDofs
                    obj.inertiaRepresentationNoise(j, k, :) = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_jaci);
                end            
            end

            obj.visionNoisePosition = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_pp);
            obj.visionNoiseVelocity = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_pv);
            obj.visionNoiseAcceleration = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_pa);

            obj.orientationNoisePosition = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_op);
            obj.orientationNoiseVelocity = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_ov);
            obj.orientationNoiseAcceleration = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_oa);

            obj.trackingErrorNoisePosition = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_bp);
            obj.trackingErrorNoiseVelocity = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_bv);
            obj.trackingErrorNoiseAcceleration = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_ba);

            obj.comSensorNoisePosition = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_cp);
            obj.comSensorNoiseVelocity = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_cv);
            obj.comSensorNoiseAcceleration = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_ca);

            obj.copSensorNoisePosition = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_cop);
            obj.torqueSensorNoise = lorentzNoise(stream, obj.totalTime, timeStep, parameters.noiseFrequencyCenter, parameters.sigma_tp);
        end
    end
end