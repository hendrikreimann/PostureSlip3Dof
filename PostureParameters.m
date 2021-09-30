% class for managing parameter values for the posture model

classdef PostureParameters
    properties
        % time
        delay_central = 0.250;               % central sensor delay, in s
        delay_mono = 0.030;                  % monosynaptic reflex delay
        delay_poly = 0.050;                  % polysynaptic reflex delay
        delay_com = 0.070;
        
        frequency = 500;                    % frequency in samples per seconds, inverse of the euler step
        eulerStep;                          % step length for numerical integration in seconds
        randomNumberSeed = 1;               % seed used for the initialization of the random number stream
        
        % body
        bodyMass = 80;                      % mass in kg
        bodyHeight = 1.80;                  % height in m
        bodyDofs;                           % degrees of freedom
        jointReference;
        muscleSetupMatrix;             % relative sizes of the muscles actuating each joint
        tau_c = 0.015;                      % calcium kinetics temporal constant
        muscle_alpha = 40;                  % in rad^(-1)
        muscle_mu = .1;                     % time constant in seconds
        muscle_cc;
        muscle_cc_ref = .01;                    % muscle cocontraction, in rad
        kneeJointLimit = 0;
        kneeJointLimitRange = -0.1;
        
        B = 0;
        B_ref = 25;
        
        % noise
        masterNoise = 1;                    % global factor for all noise
        noiseFrequencyCenter = 5;           % reference frequency for colored noise
        sigma_m = 0.02;                        % muscle noise
        sigma_n = 0.006;                      % neuronal noise
        sigma_r = 0.0;                     % reference noise
        sigma_jp = 0.002;                     % joint angle sensor noise
        sigma_jv = 0.002;                     % joint velocity sensor noise
        sigma_ja = 0.02;                    % joint acceleration sensor noise
        sigma_pp = 0.0;                     % head position position sensor noise
        sigma_pv = 0.015;                     % head position velocity sensor noise
        sigma_pa = 0.015;                    % head position acceleration sensor noise
        sigma_op = 0.02;                     % head orientation position sensor noise
        sigma_ov = 0.0;                     % head orientation velocity sensor noise
        sigma_oa = 0.0;                    % head orientation acceleration sensor noise
        sigma_bp = 0.0;                    % tracking position sensor noise
        sigma_bv = 0.0;                    % tracking velocity sensor noise
        sigma_ba = 0.0;                    % tracking acceleration sensor noise
        sigma_cp = 0.0;                     % CoM sensor noise for position
        sigma_cv = 0.02;                     % CoM sensor noise for velocity
        sigma_ca = 0.02;                     % CoM sensor noise for acceleration
        sigma_tp = 0;                     % joint torque sensor noise
        sigma_cop = 0;                       % center of pressure sensor noise
        sigma_jacm = 0;                        % CoM Jacobian representation noise
        sigma_jacv = 0;                        % head Jacobian representation noise
        sigma_jaci = 0;                        % inertia representation noise
        
        muscleNoisePower = -500;                    % noise power, in decibel per Hz, relative to 1Hz
        neuralNoisePower = -500;                    % noise power, in decibel per Hz, relative to 1Hz
        referenceNoisePower = -95;                  % noise power, in decibel per Hz, relative to 1Hz
        proprioceptionNoisePositionPower = -75;     % noise power, in decibel per Hz, relative to 1Hz
        proprioceptionNoiseVelocityPower = -75;     % noise power, in decibel per Hz, relative to 1Hz
        proprioceptionNoiseAccelerationPower = -80; % noise power, in decibel per Hz, relative to 1Hz
        visionNoisePositionPower = -75;             % noise power, in decibel per Hz, relative to 1Hz
        visionNoiseVelocityPower = -75;             % noise power, in decibel per Hz, relative to 1Hz
        visionNoiseAccelerationPower = -80;         % noise power, in decibel per Hz, relative to 1Hz
        orientationNoisePositionPower = -500;       % noise power, in decibel per Hz, relative to 1Hz
        orientationNoiseVelocityPower = -500;       % noise power, in decibel per Hz, relative to 1Hz
        orientationNoiseAccelerationPower = -500;   % noise power, in decibel per Hz, relative to 1Hz
        trackingErrorNoisePositionPower = -500;     % noise power, in decibel per Hz, relative to 1Hz
        trackingErrorNoiseVelocityPower = -500;     % noise power, in decibel per Hz, relative to 1Hz
        trackingErrorNoiseAccelerationPower = -500; % noise power, in decibel per Hz, relative to 1Hz
        comSensorNoisePositionPower = -50;          % noise power, in decibel per Hz, relative to 1Hz
        comSensorNoiseVelocityPower = -70;          % noise power, in decibel per Hz, relative to 1Hz
        comSensorNoiseAccelerationPower = -70;      % noise power, in decibel per Hz, relative to 1Hz
        copSensorNoisePositionPower = -80;          % noise power, in decibel per Hz, relative to 1Hz
        torqueSensorNoisePower = -15;               % noise power, in decibel per Hz, relative to 1Hz

        % estimation low pass filter gain
        beta_pp_spinal = 20;                % low pass filter gain for spinal estimate of joint position
        beta_pv_spinal = 20;                % low pass filter gain for spinal estimate of joint velocity
        beta_pw_spinal = 20;                % low pass filter gain for spinal estimate of joint acceleration
        beta_pp_central = 20;               % low pass filter gain for central estimate of joint position
        beta_pv_central = 20;               % low pass filter gain for central estimate of joint velocity
        beta_mp = 20;                       % low pass filter gain for central estimate CoM position
        beta_mv = 20;                       % low pass filter gain for central estimate CoM velocity
        beta_cp = 1;
        beta_pt = 5;

        % forcelet switches
        alpha_j = 1;
        alpha_c = 0;
        alpha_p = 0;
        alpha_o = 0;
        alpha_b = 0;
        alpha_t = 0;
        
        % joint forcelet parameters
        alpha_jpl = 1;
        alpha_jpq = 0;
        alpha_jvl = 15;
        alpha_jvq = 0;
        alpha_jal = 1;
        
        % head position forcelet parameters
        alpha_pp = 0;
        alpha_pv = 0;
        alpha_pa = 0;
        alpha_pvq = 0;
        alpha_paq = 0;

        % head orientation forcelet parameters
        alpha_op = 0.1;
        alpha_ov = 0;
        alpha_oa = 0;

        % tracking task forcelet parameters
        alpha_bp = 0.0;
        alpha_bv = 300;
        alpha_ba = 20;
        
        % com forcelet parameters
        alpha_cp = 1.0;
        alpha_cv = 1.0;
        alpha_ca = 0.0;
        
        % torque forcelet parameters
        alpha_tp = 3.0;
        alpha_tv = 5.0;

        % back-coupling parameters
        alpha_bc = 0;
    end
    methods
        function obj = PostureParameters(bodyDofs)
            % time
            obj.eulerStep = 1 / obj.frequency;
            
            % body
            obj.bodyDofs = bodyDofs;
            obj.jointReference = zeros(bodyDofs, 1);
            obj.muscleSetupMatrix = eye(bodyDofs);
            obj.muscle_cc = zeros(bodyDofs, 1);

        end % constructor

        function stiffnessValues = stiffnessValuesFromFraction( obj, plant )
            stiffnessValues = obj.K_f .* plant.mPassiveStabilityStiffnessThresholds;
        end
    end
end














