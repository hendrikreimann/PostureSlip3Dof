% simulates postural control for multi-DoF manipulator with muscle model

% Hendrik Reimann, 2009-9 - present

%#ok<*DEFNU>

% function [plant, postureTimeSeries, parameters] = simulation(arg)
function simulation(condition, trialParameter, seed)

%% prepare and set parameters
T = 35;
bodyDofs = 3;
parameters = PostureParameters(bodyDofs);

% declare some parameters
global index;
index = 1;
headReference_pos = []; headReference_ori = []; copReference = []; comReference = []; parameters.jointReference = []; currentParameter = [];
d_E_by_d_lambda = eye(bodyDofs);

% exploration stuff
useInitialPerturbation = 0; % 1 = joint angle, 2 = joint velocity, 3 = head orientation reference
useAnkleStrategy = 0;
usePurelyLocalStrategy = 0;
parameters.masterNoise = 1;
parameters.delay_central = 0.120;
parameters.delay_poly = 0.030;
parameters.delay_com = 0.120;
% parameters.delay_poly = 0.140; % to check out the response to the Loram Lakie perturbation paradigm

% % reduced delays for testing
% useInitialPerturbation = 3; % 1 = joint angle, 2 = joint velocity, 3 = head orientation reference
% parameters.masterNoise = 0;
% parameters.delay_central = 0.100;
% parameters.delay_poly = 0.010;
% parameters.delay_com = 0.100;
% parameters.tau_c = 0.01;                  % in rad^(-1)

parameters.muscle_alpha = 12.0;                  % in rad^(-1)
parameters.muscle_mu = 0.06;                     % time constant in seconds
parameters.muscle_cc_ref = .01;                    % muscle cocontraction, in rad

parameters.muscleSetupMatrix = [10.94 1.1 0; ...
                                0 7.43 1.2; ...
                                0.0 0.94 9.10];
parameters.muscleSetupMatrix = parameters.muscleSetupMatrix(1:bodyDofs, 1:bodyDofs);


% parameters.sigma_n = 0.0;
% parameters.sigma_m = 0.0;



parameters.alpha_j = 0;
parameters.alpha_c = 0;
parameters.alpha_p = 1;
parameters.alpha_o = 1;

damping_factor = 0.5;


% joint-wise control on velocity level
parameters.alpha_jpl = 0;
parameters.alpha_jvl = 18;
parameters.alpha_jal = damping_factor * 2 * sqrt(parameters.alpha_jvl);

parameters.alpha_cp = 0;
parameters.alpha_cv = 1.5;
% parameters.alpha_ca = 0.5;
parameters.alpha_ca = damping_factor * 2 * sqrt(parameters.alpha_cv);

parameters.alpha_pp = 0;
parameters.alpha_pv = 12;
parameters.alpha_pa = damping_factor * 2 * sqrt(parameters.alpha_pv);
parameters.alpha_pvq = 0;
parameters.alpha_paq = damping_factor * 2 * sqrt(parameters.alpha_pvq);

parameters.alpha_op = 40;
parameters.alpha_ov = 0;
parameters.alpha_oa = 0;%damping_factor * 2 * sqrt(parameters.alpha_ov);

parameters.randomNumberSeed = 2;




% XXX check out the ankle strategy with stiffened upper joints
% parameters.muscle_cc = [0.01; 0.15; 0.15];
% parameters.alpha_j = 0;
% parameters.alpha_c = 0;
% parameters.alpha_p = 1;
% parameters.alpha_o = 0;
% 
% parameters.delay_poly = 0.0;
% parameters.delay_central = 0.0;
% useAnkleStrategy = 1;
% usePurelyLocalStrategy = 1;
% parameters.alpha_pv = 5;
% parameters.alpha_pa = .1;
% parameters.alpha_pv = -5 * 1.6^(-1);
% parameters.alpha_pa = -.1 * 1.6^(-1);
% parameters.alpha_pv = -3.125;
% parameters.alpha_pa = -0.0625;
% % alpha_pa = damping_factor * 2 * sqrt(-parameters.alpha_pv);
% % parameters.alpha_pa = - damping_factor * 2 * sqrt(-parameters.alpha_pv);
% parameters.alpha_pv = -3;
% parameters.alpha_pa = 0;


% XXX check out a joint level strategy on position level
% parameters.alpha_j = 1;
% parameters.alpha_c = 0;
% parameters.alpha_p = 1;
% parameters.alpha_o = 0;
% 
% usePurelyLocalStrategy = 1;
% useAnkleStrategy = 1;
% 
% parameters.alpha_jpl = 0.5;
% parameters.alpha_jvl = 2;
% parameters.alpha_pv = 1;
% parameters.alpha_pa = 1;

% parameters.alpha_jpl = 250;
% parameters.alpha_jvl = 150;
% parameters.alpha_jvl = damping_factor * 2 * sqrt(parameters.alpha_jpl);
% a = parameters.alpha_jvl;

% parameters.delay_central = 0.0;
% parameters.delay_poly = 0.0;


% XXX check out ankle plus cocontraction strategy
%         parameters.alpha_j = 0;
%         parameters.alpha_c = 0;
%         parameters.alpha_p = 1;
%         parameters.alpha_o = 0;
%         
%         parameters.muscle_cc = [0.01; 0.15; 0.15];
%         parameters.delay_poly = 0.0;
%         parameters.delay_central = 0.0;
%         useAnkleStrategy = 1;
%         usePurelyLocalStrategy = 1;
%         parameters.alpha_pv = 5;
%         parameters.alpha_pa = .1;



% XXX - turn off cortical feedback
% parameters.alpha_c = 0;
% parameters.alpha_p = 0;
% parameters.alpha_o = 0;

% XXX - multiply the feedback gains to check out extreme values of muscle_alpha
% factor = 1;
% parameters.alpha_c = parameters.alpha_c * factor;
% parameters.alpha_p = parameters.alpha_p * factor;
% parameters.alpha_o = parameters.alpha_o * factor;

% check whether function was called from outside with parameter set provided
if nargin < 1
    withGui = 1;
else
    useInitialPerturbation = 0;
    parameters.masterNoise = 1;
    T = 35;
%     T = 5; % XXX
    parameters.randomNumberSeed = seed;

    % adjusting the noise strength parameters
    parameters.sigma_jp = 0.002;
    parameters.sigma_jv = 0.005;
    parameters.sigma_ja = 0.02;

    parameters.alpha_pv = 15;
    parameters.alpha_pa = damping_factor * 2 * sqrt(parameters.alpha_pv);

    parameters.sigma_n = 0.0;
    parameters.sigma_m = 0.0;
    
    
    
    if condition==1 % joint feedback
        parameters.alpha_j = 1;
        parameters.alpha_c = 0;
        parameters.alpha_p = 0;
        parameters.alpha_o = 0;
    elseif condition==2 % head position feedback, eyes open
        useAnkleStrategy = 0;

        parameters.alpha_j = 0;
        parameters.alpha_c = 0;
        parameters.alpha_p = 1;
        parameters.alpha_o = 1;

        parameters.sigma_pv = 0.007;
        parameters.sigma_pa = 0.03;
        parameters.sigma_op = 0.025;
        parameters.sigma_n = 0.001;
        
        
        parameters.alpha_pv = 12;
        parameters.alpha_pa = damping_factor * 2 * sqrt(parameters.alpha_pv);

        parameters.alpha_op = 40;
        
    elseif condition==3 % head position feedback, eyes closed
        useAnkleStrategy = 0;

        parameters.alpha_j = 0;
        parameters.alpha_c = 0;
        parameters.alpha_p = 1;
        parameters.alpha_o = 1;

        parameters.sigma_pv = 0.01;
        parameters.sigma_pa = 0.032;
        parameters.sigma_op = 0.032;
        parameters.sigma_n = 0.001;
        
        
        parameters.alpha_pv = 12;
        parameters.alpha_pa = damping_factor * 2 * sqrt(parameters.alpha_pv);

        parameters.alpha_op = 40;
        
    elseif condition==4 % high-level ankle strategy, head vel/acc into ankle, joint pos/vel into joints
        parameters.alpha_j = 1;
        parameters.alpha_c = 0;
        parameters.alpha_p = 1;
        parameters.alpha_o = 0;

        usePurelyLocalStrategy = 0;
        useAnkleStrategy = 1;
        parameters.sigma_n = 0.001;
        parameters.alpha_jpl = 5;
        parameters.alpha_jvl = 0;
        parameters.alpha_pv = 15;
        parameters.alpha_pa = damping_factor * 2 * sqrt(parameters.alpha_pv);
%         parameters.delay_central = 0.0;
%         parameters.delay_poly = 0.0;
        % --------------------------------------------------------
        parameters.alpha_j = 1;
        parameters.alpha_c = 0;
        parameters.alpha_p = 1;
        parameters.alpha_o = 0;

        usePurelyLocalStrategy = 1;
        useAnkleStrategy = 1;

        parameters.sigma_n = 0.001;

        parameters.alpha_jpl = 0.5;
        parameters.alpha_jvl = 2;
        parameters.alpha_pv = 1;
        parameters.alpha_pa = 1;

        parameters.delay_central = 0.0;
        parameters.delay_poly = 0.0;    
    
    
    elseif condition==5 % head position feedback, high-level ankle strategy with coordination, eyes closed
        useAnkleStrategy = 1;

        parameters.alpha_j = 0;
        parameters.alpha_c = 0;
        parameters.alpha_p = 1;
        parameters.alpha_o = 1;

        parameters.sigma_pv = 0.007;
        parameters.sigma_pa = 0.03;
        parameters.sigma_op = 0.025;
        parameters.sigma_n = 0.001;
        
        
        parameters.alpha_pv = 12;
        parameters.alpha_pa = damping_factor * 2 * sqrt(parameters.alpha_pv);

        parameters.alpha_op = 40;
    elseif condition==6 % low-level feedback ankle strategy
        parameters.alpha_j = 0;
        parameters.alpha_c = 0;
        parameters.alpha_p = 1;
        parameters.alpha_o = 0;
        
        parameters.muscle_cc = [0.01; 0.15; 0.15];
        parameters.delay_poly = 0.0;
        parameters.delay_central = 0.0;
        useAnkleStrategy = 1;
        usePurelyLocalStrategy = 1;
        parameters.alpha_pv = 5;
        parameters.alpha_pa = .1;
%         parameters.alpha_pv = -2;
%         parameters.alpha_pa = 0;
        
        
    elseif condition==7 % high-level feedback joint strategy
        usePurelyLocalStrategy = 1;
        
        parameters.sigma_n = 0.001;
        parameters.alpha_jpl = 25;
        parameters.alpha_jvl = 25;
        parameters.alpha_jal = 0;
        parameters.delay_central = 0.0;
        parameters.delay_poly = 0.0;
        parameters.delay_com = 0.0;
    end
    
    
    parameters.sigma_jp = parameters.sigma_jp * trialParameter;
    parameters.sigma_jv = parameters.sigma_jv * trialParameter;

%     parameters.sigma_m = trialParameter;
    

    
    
    currentParameter = trialParameter;
%     parameters.alpha_pv = trialParameter;
%     parameters.alpha_pa = damping_factor * 2 * sqrt(parameters.alpha_pv);

    withGui = 0;
    
    source = mfilename('fullpath');
    location = pwd;
    copyfile([source, '.m'], [location '/_simulation.m']);
end
%     withGui = 1;

% parameters.sigma_pv = 0;
% parameters.sigma_pa = 0;


% for checking out white noise
% parameters.sigma_jp = 0;
% parameters.sigma_jv = 0;
% parameters.sigma_pv = 0;
% parameters.sigma_pa = 0;
% parameters.sigma_op = 0;
% parameters.sigma_m = 0;
% parameters.sigma_n = 0;



% reset random number stream using the provided seed and generate noise
stream = RandStream('mt19937ar');
% stream = RandStream('mrg32k3a');
reset(stream, parameters.randomNumberSeed);
noise = PostureNoise(parameters.frequency, T, bodyDofs);
% noise = noise.generateWhiteNoise(parameters, stream);
% noise = noise.generatePinkNoise(parameters);
noise = noise.generateColoredNoise(parameters, stream);

%% setup
% disable some warnings
%#ok<*UNRCH>

parameters.jointReference = [-.1; .2; -.2; -0.05; 0.1; -0.2];
parameters.jointReference = parameters.jointReference(1:bodyDofs);


parameters.B = parameters.muscleSetupMatrix * parameters.B_ref / parameters.muscleSetupMatrix(1, 1);

% physiology
useRelativeDamping = 1;
useCalciumKinetics = 1;
useBoundedActivation = 0;

% program stuff
isDrawing = false;

% visual drive
addVisualDrive_slow = false;
addVisualDrive_fast = false;
DEGTORAD = pi/180;
visualDriveFrequency_slow = 0.2;
visualDriveAmplitude_slow = DEGTORAD*0.2 * addVisualDrive_slow;
visualDriveFrequency_fast = 2.0;
visualDriveAmplitude_fast = DEGTORAD*0.028 * addVisualDrive_fast;

% model
useContactDofs = 0;
useKneeJointLimit = false;
kneeJointConstraintVersion = 3; % 1 = absolutely rigid, 2 = rigid except first breach time step, 3 = deforming

% force plate displacement
applyDisplacement = 0;
if applyDisplacement
    useContactDofs = 1;
    
    % anterior-posterior displacement
    displacementOnsetTime = 2.0;
    displacementTime = 0.28;
    displacement = 0.06;
    displacementDof = 1;
    displacementType = 0; % 0 = ramp, 1 = sinusoid, 2 = transient sinusoid

    % plane inclination, Kluzik 2005 paradigm
%     displacementOnsetTime = 0.3;
%     displacementTime = 5;
%     displacement = DEGTORAD * (-5);
%     displacementDof = 3;
%     displacementType = 0; % 0 = ramp, 1 = sinusoid, 2 = transient sinusoid
    
    % plane inclination, Loram Lakie 2002 paradigm
%     displacementOnsetTime = 0.1;
%     displacementTime = 0.14;
%     displacement = - DEGTORAD * 0.055;
%     displacementDof = 3;
%     displacementType = 2; % 0 = ramp, 1 = sinusoid, 2 = transient sinusoid
    
end

% voluntary movement
doVoluntaryMovement = 0;
voluntaryMovementOnsetTime = 0.5;
voluntaryMovementTime = 0.5;
voluntaryMovementMagnitude = 0.1;
voluntaryMovementType = 3; % 0 = bend knees, 1 = lean backward, 2 = ankle, 3 - ankle/hip

% secondary task
useSecondaryTask = false;
secondaryTask_frequency = 0.3;
secondaryTask_floor = 1.5;
secondaryTask_ceiling = 1.8;
viewDirection = zeros(3, 1);

%% initialize

% objects
if (useContactDofs || bodyDofs>3)
%     disp('using screw version of posture plant');
    plant = PosturePlant(parameters.bodyDofs, useContactDofs, 1.8, 80);
else
%     disp('using explicit version of posture plant');
    plant = PosturePlant_symbolic(parameters.bodyDofs, 1.8, 80);
end
if (bodyDofs<=3)
    internalModel = PosturePlant_symbolic(parameters.bodyDofs, 1.8, 80);
else
    internalModel = PosturePlant(parameters.bodyDofs, useContactDofs, 1.8, 80);
end


postureTimeSeries = PostureTimeSeries(parameters.frequency, T*parameters.frequency, plant, parameters.bodyDofs);
N = plant.mNumberOfJoints;

init;


% plots and GUI
if withGui
    stickFigure = StickFigure(plant);
    timeSeriesFigure = PostureTimeSeriesFigure(postureTimeSeries);
    plotVisibilityCheckBoxes = createControlFigure();
    if useSecondaryTask
        stickFigure.update ...
        ( ...
            plant, ...
            postureTimeSeries.q(1), ...
            postureTimeSeries.b(1), ...
            secondaryTask_floor, ...
            secondaryTask_ceiling ...
        );
    else
        stickFigure.update(plant); 
    end
    timeSeriesFigure.mIsDrawingLambda = 0;
    timeSeriesFigure.update(postureTimeSeries, index);
    updatePlotVisibility(0, 0);
end

% tic
runSimulation(0, 0);
% toc

% save plant.mat plant;
% save parameters.mat parameters;
% save noise.mat noise;
% save postureTimeSeries.mat postureTimeSeries;

%% main

function init()
    % parameters.jointReference = [ -0.1127;    0.0492;    0.0344;    0.1084;   -0.5322 ]; % mean of subject MM, first trial
    % parameters.jointReference = [ -0.2097;    0.2167;    0.4086;   -0.2767;   -0.9089;   -0.1715 ]; % first time step of s1eo03_jan_rev.mat
%     parameters.jointReference = zeros(6, 1);
    
    parameters.jointReference = parameters.jointReference(1:parameters.bodyDofs);
    plant.mJointAngles(plant.mContactDofs+1:end) = parameters.jointReference;
    plant = plant.updateInternals();
    
    % find stable lambda configuration
    [lambda, steadyStateTorque] = stableLambda_EPH(plant, parameters);
    
%     theta_virtual = findVirtualConfiguration(parameters, lambda);
    
    postureTimeSeries.lambdaTilde(:, 1) = lambda;
    postureTimeSeries.lambda(:, 1) = postureTimeSeries.lambdaTilde(:, 1);
    postureTimeSeries.muscleTorque(:, 1) = steadyStateTorque;
    
    
    
    postureTimeSeries.rho(:, 1) = parameters.jointReference;
    
    % define references
    headReference_pos = plant.mEndEffectorPosition;
    headReference_ori = plant.mEndEffectorOrientation;
    copReference = 0;
    postureTimeSeries.q(1) = plant.mEndEffectorPosition(3);

    % initialize variables properly
    postureTimeSeries.theta(plant.mContactDofs+1:end, 1) = parameters.jointReference;
    postureTimeSeries.xJoint(:, 1) = parameters.jointReference;
    postureTimeSeries.uJoint_mono(:, 1) = parameters.jointReference;
    postureTimeSeries.uJoint_poly(:, 1) = parameters.jointReference;
    postureTimeSeries.uJoint_central(:, 1) = parameters.jointReference;
    postureTimeSeries.xHeadPosition(:, 1) = plant.mEndEffectorPosition(1:3);
    postureTimeSeries.uHeadPosition(:, 1) = plant.mEndEffectorPosition(1:3);
    postureTimeSeries.headPosition_p(:, 1) = plant.mEndEffectorPosition(1:3);
    postureTimeSeries.xHeadOrientation(1) = plant.mEndEffectorOrientation;
    postureTimeSeries.uHeadOrientation(1) = plant.mEndEffectorOrientation;
    postureTimeSeries.headOrientation_p(1) = plant.mEndEffectorOrientation;
    comReference = [0; 0; 0];
%     for i = plant.mContactDofs+1 : N
%         comReference = comReference + plant.mLinkTransformations{i}(1:3, 4)*plant.mLinkMasses(i);
%     end
%     comReference = comReference * (1 / parameters.bodyMass);
    comReference = plant.mCom;
    postureTimeSeries.com(:, 1) = comReference;
    postureTimeSeries.xCom(:, 1) = comReference(2);
    postureTimeSeries.uCom(:, 1) = comReference(2);
    postureTimeSeries.cop(1) = postureTimeSeries.com(2, 1);
    postureTimeSeries.xCop(1) = postureTimeSeries.com(2, 1);
    postureTimeSeries.uCom(:, 1) = postureTimeSeries.com(2, 1);
    
    postureTimeSeries.uTorque(:, 1) = postureTimeSeries.muscleTorque(:, 1);
    % apply perturbation
    if useInitialPerturbation == 1 % perturb joint angle
        plant.mJointAngles(plant.mContactDofs+2) = plant.mJointAngles(plant.mContactDofs+2) - 0.0005;
    elseif useInitialPerturbation == 2 % perturb joint velocity
        plant.mJointVelocities(plant.mContactDofs+1) = plant.mJointVelocities(plant.mContactDofs+1) - 0.005;
%         plant.mJointVelocities(plant.mContactDofs+2) = plant.mJointVelocities(plant.mContactDofs+2) - 0.005;
%         plant.mJointVelocities(plant.mContactDofs+3) = plant.mJointVelocities(plant.mContactDofs+2) - 0.005;
    elseif useInitialPerturbation == 3 % perturb head orientation reference
        headReference_ori = headReference_ori + 0.2;
    end

end
function runSimulation(hObject, eventdata) %#ok<INUSD>
    
    % pregenerate a vector of time steps to loop through. First one is now,
    % so start with second step
    timeSteps = 2 : length(postureTimeSeries.time);
    
    % if we do not want life update, we want a progress bar
    if (isDrawing == false) && withGui
%         progressBar = waitbar(0, 'Trying not to fall down...', 'Position', [ 1200, 1600, 400, 60 ]);
        progressBar = waitbar(0, 'Trying not to fall down...');
    end
    
    exploded = false;
    
    % simulation loop
%     fprintf(['simulating ' num2str(length(postureTimeSeries.time)) ' time steps, finished ']);
    for index = timeSteps %#ok<FXUP>
        
        if (exploded == false)
            % sensors
            updateJointSensorEstimates(index);
            updateHeadPositionSensorEstimates(index);
            updateHeadOrientationSensorEstimates(index);
            updateTrackingErrorSensorEstimates(index);
            updateComEstimates(index);
            updateJointTorqueEstimates(index);
            internalModel.mJointAngles = postureTimeSeries.uJoint_central(:, index);
            internalModel.mJointVelocities = postureTimeSeries.vJoint_central(:, index);
            internalModel = internalModel.updateInternals();
            
            postureTimeSeries.J_head(:, index) = internalModel.mEndEffectorJacobian(2, :)';
            postureTimeSeries.J_com(:, index) = internalModel.mComJacobian(2, :)';
            postureTimeSeries.inertiaMatrix(:, :, index) = internalModel.mInertiaMatrix;
            

            % neural dynamics
            updateRho(index);
            updateLambda(index);

            % muscle torque
            
            updateMuscleActivation(index);
            updateMuscleTorques(index);
            
            theta_passive_rad = plant.mJointAngles(plant.mContactDofs+1:end);
            theta_passive_deg = rad2deg(theta_passive_rad);
            theta_passive_deg(3) = - theta_passive_deg(3);
            stiffnessTorque_1 = exp(2.1016 - 0.0843*theta_passive_deg(1) - 0.0176*theta_passive_deg(2)) - exp(- 7.9763 + 0.1949*theta_passive_deg(1) + 0.0008*theta_passive_deg(2)) - 1.792;
            stiffnessTorque_2 = exp(1.800 - 0.0460*theta_passive_deg(1) - 0.0352*theta_passive_deg(2) + 0.0217*theta_passive_deg(3)) - exp(-3.971 - 0.0004*theta_passive_deg(1) + 0.0495*theta_passive_deg(2) - 0.0128*theta_passive_deg(3)) - 4.820 + exp(2.220 - 0.150*theta_passive_deg(2));
            stiffnessTorque_3 = exp(1.4655 - 0.0034*theta_passive_deg(2) - 0.075*theta_passive_deg(3)) - exp(1.3403 - 0.0226*theta_passive_deg(2) + 0.0305*theta_passive_deg(3)) + 8.072;
            factor = 1;
            postureTimeSeries.stiffnessTorque(:, index) = factor*[stiffnessTorque_1; stiffnessTorque_2; stiffnessTorque_3];
%             maximalTorque = 100;
%             for i_joint = 1 : bodyDofs
%                 if abs(postureTimeSeries.stiffnessTorque(i_joint, index)) > maximalTorque
%                     postureTimeSeries.stiffnessTorque(i_joint, index) = maximalTorque * sign(postureTimeSeries.stiffnessTorque(i_joint, index));
%                 end
%             end
            
            
            
            
            postureTimeSeries.dampingTorque(:, index) = - parameters.B * plant.mJointVelocities(plant.mContactDofs+1:end);
            postureTimeSeries.appliedTorque(:, index) = postureTimeSeries.muscleTorque(:, index) ...
                                                        + postureTimeSeries.stiffnessTorque(:, index) ...
                                                        + postureTimeSeries.dampingTorque(:, index);
                                                    
    
                                                    
            
            constraintForces = calculateConstraintForces(index);
                          
            tau = [zeros(plant.mContactDofs, 1); postureTimeSeries.appliedTorque(:, index)];
            plant.mExternalTorques = tau + constraintForces;
            
            % update plant
            plant.mJointAccelerations = plant.mInertiaMatrix^(-1) * ...
                                          (plant.mExternalTorques ...
                                            - plant.mGravitationalTorques ...
                                            - plant.mCoriolisMatrix * plant.mJointVelocities ...
                                         );
                                     
            plant.mJointVelocities = plant.mJointVelocities + parameters.eulerStep * plant.mJointAccelerations;
            plant.mJointAngles = plant.mJointAngles + parameters.eulerStep * plant.mJointVelocities;
            plant = plant.updateInternals();
            
            % update secondary task
            if useSecondaryTask
                amplitude = secondaryTask_ceiling - secondaryTask_floor;
                time = postureTimeSeries.time(index);
                postureTimeSeries.q(index) = secondaryTask_floor + ...
                                               amplitude * (cos(time*2*pi*secondaryTask_frequency) + 1) / 2;
                postureTimeSeries.qDot(index) = - amplitude * sin(time*2*pi*secondaryTask_frequency) / 2;
                postureTimeSeries.qTwoDot(index) = - amplitude * cos(time*2*pi*secondaryTask_frequency) / 2;
                
                
                
                T = plant.mJointTransformations{end};
                headPosition = T(1:3, 4);
                viewDirection = T(1:3, 2);
                lambda = (2 - headPosition(2)) / viewDirection(2);
                postureTimeSeries.b(index) = headPosition(3) + lambda*viewDirection(3);
                postureTimeSeries.bDot(index) = 1 / parameters.eulerStep ...
                                                  * (postureTimeSeries.b(index) - postureTimeSeries.b(index-1));
                postureTimeSeries.bTwoDot(index) = 1 / parameters.eulerStep ...
                                                     * (postureTimeSeries.bDot(index) - postureTimeSeries.bDot(index-1));
            end
            
            
            % export data to time series object
            postureTimeSeries.externalTorque(:, index) = plant.mExternalTorques;
            postureTimeSeries.gravitationalTorque(:, index) = plant.mGravitationalTorques;
            postureTimeSeries.theta(:, index) = plant.mJointAngles;
            postureTimeSeries.thetaDot(:, index) = plant.mJointVelocities;
            postureTimeSeries.thetaTwoDot(:, index) = plant.mJointAccelerations;
            postureTimeSeries.headPosition_p(:, index) = plant.mEndEffectorPosition(1:3);
            postureTimeSeries.headPosition_v(:, index) = plant.mEndEffectorVelocity(1:3);
%             postureTimeSeries.headPosition_a(:, index) = 1/parameters.eulerStep ...
%                                               * (postureTimeSeries.headPosition_v(:, index) - postureTimeSeries.headPosition_v(:, index-1));
%             a1 = postureTimeSeries.headPosition_a(:, index);
%             a2 = plant.mEndEffectorAcceleration;
            postureTimeSeries.headPosition_a(:, index) = plant.mEndEffectorAcceleration;
            postureTimeSeries.headOrientation_p(:, index) = plant.mEndEffectorOrientation;
            J_ori = ones(1, bodyDofs);
            postureTimeSeries.headOrientation_v(:, index) = J_ori * plant.mJointVelocities(plant.mContactDofs+1:end);
            postureTimeSeries.headOrientation_a(:, index) = J_ori * plant.mJointAccelerations(plant.mContactDofs+1:end);

            % center of mass
            postureTimeSeries.com(:, index) = plant.mCom;
            postureTimeSeries.comDot(:, index) = 1/parameters.eulerStep ...
                                              * (postureTimeSeries.com(:, index) - postureTimeSeries.com(:, index-1));
            postureTimeSeries.comTwoDot(:, index) = 1/parameters.eulerStep ...
                                              * (postureTimeSeries.comDot(:, index) - postureTimeSeries.comDot(:, index-1));
            
            % center of pressure
            if useContactDofs
                % see Winter, Biomechanics, p. 118
                F_y = constraintForces(1); % horizontal force at (0, 0, 0) in anterior-posterior direction
                F_z = constraintForces(2); % vertical ground reaction force at (0, 0, 0)
                M_x = constraintForces(3); % torque around x-axis through (0, 0, z_s) = location of hypothetical sensor
                z_s = plant.mReferenceJointTwists{3}(2); % location of hypothetical sensor on z-axis
                % cop is calculated relative to the rotational contact DoF, apply an offset to transform to world coords
                offset = plant.mReferenceJointTransformations{1}(2, 4); 
                postureTimeSeries.cop(index) = (F_y*z_s + M_x) / F_z + offset;
                
                postureTimeSeries.copDot(index) = 1/parameters.eulerStep ...
                                                  * (postureTimeSeries.cop(index) - postureTimeSeries.cop(index-1));
                postureTimeSeries.copTwoDot(index) = 1/parameters.eulerStep ...
                                                  * (postureTimeSeries.copDot(index) - postureTimeSeries.copDot(index-1));
                postureTimeSeries.groundReactionForces(:, index) = constraintForces(1:plant.mContactDofs);
            else
                postureTimeSeries.cop(index) = copReference;
            end
            
            % check for explosion
            if (max(abs(plant.mJointAngles(plant.mContactDofs+1:end) - parameters.jointReference)) > 5.0)
                exploded = true;
                disp('exploded!');
            end
            
            if withGui
                if (isDrawing)
                    updatePlots(index);
                else
                    if ((index / 100) == floor(index / 100))
                        waitbar(index / length(timeSteps), progressBar, 'Trying not to fall down...')
                    end
                end
            end
        else
            % write zeroes
            postureTimeSeries.uJoint_mono(:, index) = zeros(parameters.bodyDofs, 1);
            postureTimeSeries.vJoint_mono(:, index) = zeros(parameters.bodyDofs, 1);
            postureTimeSeries.uJoint_poly(:, index) = zeros(parameters.bodyDofs, 1);
            postureTimeSeries.vJoint_poly(:, index) = zeros(parameters.bodyDofs, 1);
            postureTimeSeries.lambda(:, index) = zeros(parameters.bodyDofs, 1);
            postureTimeSeries.thetaTwoDot(:, index) = zeros(N, 1);
            postureTimeSeries.thetaDot(:, index) = zeros(N, 1);
            postureTimeSeries.theta(:, index) = zeros(N, 1);
            postureTimeSeries.externalTorque(:, index) = zeros(N, 1);
            postureTimeSeries.headPosition_p(:, index) = zeros(3, 1);
            postureTimeSeries.headPosition_v(:, index) = zeros(3, 1);
            postureTimeSeries.headPosition_a(:, index) = zeros(3, 1);
            postureTimeSeries.headOrientation_p(:, index) = 0;
            postureTimeSeries.headOrientation_v(:, index) = 0;
            postureTimeSeries.headOrientation_a(:, index) = 0;
            postureTimeSeries.lambdaDot(:, index) = zeros(parameters.bodyDofs, 1);
            postureTimeSeries.com(:, index) = zeros(3, 1);
            postureTimeSeries.cop(:, index) = 0;
        end

    end % simulation loop
    
    % if calculating blindly, update plots now
    if (isDrawing == false) && withGui 
        close(progressBar);
        updatePlots(index)
    end

    fprintf('finished simulation');
    if ~isempty(currentParameter)
        fprintf([', parameter: ' num2str(currentParameter)]);
        fprintf([', seed: ' num2str(seed)]);
    end
    fprintf('\n');
end % runSimulation

%% sensor dynamics

function updateJointSensorEstimates(index)
    proprioceptionNoisePosition = parameters.masterNoise * noise.proprioceptionNoisePosition(:, index);
    proprioceptionNoiseVelocity = parameters.masterNoise * noise.proprioceptionNoiseVelocity(:, index);
    proprioceptionNoiseAcceleration = parameters.masterNoise * noise.proprioceptionNoiseAcceleration(:, index);
    
    % sensor data of joint variables
    postureTimeSeries.xJoint(:, index) = plant.mJointAngles(plant.mContactDofs+1:end) + proprioceptionNoisePosition;
    postureTimeSeries.yJoint(:, index) = plant.mJointVelocities(plant.mContactDofs+1:end) + proprioceptionNoiseVelocity;
    postureTimeSeries.zJoint(:, index) = plant.mJointAccelerations(plant.mContactDofs+1:end) + proprioceptionNoiseAcceleration;

    % use sensor data with delay
    delay_monoTimeSteps = parameters.delay_mono * parameters.frequency;
    index_mono = index - delay_monoTimeSteps;
    if index_mono < 1
        index_mono = 1;
    end
    delay_polyTimeSteps = parameters.delay_poly * parameters.frequency;
    index_poly = index - delay_polyTimeSteps;
    if index_poly < 1
        index_poly = 1;
    end
    delay_centralTimeSteps = parameters.delay_central * parameters.frequency;
    index_central = index - delay_centralTimeSteps;
    if index_central < 1
        index_central = 1;
    end
    
    % update estimates directly
    postureTimeSeries.uJoint_mono(:, index) = postureTimeSeries.xJoint(:, index_mono);
    postureTimeSeries.vJoint_mono(:, index) = postureTimeSeries.yJoint(:, index_mono);
    postureTimeSeries.wJoint_mono(:, index) = postureTimeSeries.zJoint(:, index_mono);
    postureTimeSeries.uJoint_poly(:, index) = postureTimeSeries.xJoint(:, index_poly);
    postureTimeSeries.vJoint_poly(:, index) = postureTimeSeries.yJoint(:, index_poly);
    postureTimeSeries.wJoint_poly(:, index) = postureTimeSeries.zJoint(:, index_poly);
    postureTimeSeries.uJoint_central(:, index) = postureTimeSeries.xJoint(:, index_central);
    postureTimeSeries.vJoint_central(:, index) = postureTimeSeries.yJoint(:, index_central);
    postureTimeSeries.wJoint_central(:, index) = postureTimeSeries.zJoint(:, index_central);
end
function updateHeadPositionSensorEstimates(index)
    visionNoisePosition =  parameters.masterNoise * noise.visionNoisePosition(:, index);
    visionNoiseVelocity =  parameters.masterNoise * noise.visionNoiseVelocity(:, index);
    visionNoiseAcceleration =  parameters.masterNoise * noise.visionNoiseAcceleration(:, index);

    % sensor data of head variables
    postureTimeSeries.xHeadPosition(:, index) = postureTimeSeries.headPosition_p(:, index-1) + visionNoisePosition;
    postureTimeSeries.yHeadPosition(:, index) = postureTimeSeries.headPosition_v(:, index-1) + visionNoiseVelocity;
    postureTimeSeries.zHeadPosition(:, index) = postureTimeSeries.headPosition_a(:, index-1) + visionNoiseAcceleration;
    
    % sensor delay
    delayTimeSteps = parameters.delay_central * parameters.frequency;
    sensorIndex = index - delayTimeSteps;
    if sensorIndex < 1
        sensorIndex = 1;
    end
    
    % calculate visual drive
    t = postureTimeSeries.time(sensorIndex);
    angularVisualDrive_vel = visualDriveAmplitude_slow * sin(t*2*pi*visualDriveFrequency_slow) ...
                             + visualDriveAmplitude_fast * sin(t*2*pi*visualDriveFrequency_fast);
    cartesianVisualDrive_vel = plant.mEndEffectorJacobian(:, plant.mContactDofs+1) * angularVisualDrive_vel;
    
    % estimate according to known delay
%     postureTimeSeries.uHeadPosition(:, index) = postureTimeSeries.xHeadPosition(:, sensorIndex) ...
%                                           + parameters.delay_central * postureTimeSeries.yHeadPosition(:, sensorIndex);
%     postureTimeSeries.vHeadPosition(:, index) = postureTimeSeries.yHeadPosition(:, sensorIndex) ...
%                                           + cartesianVisualDrive_vel ...
%                                           + parameters.delay_central * postureTimeSeries.zHeadPosition(:, sensorIndex);
%     postureTimeSeries.wHeadPosition(:, index) = postureTimeSeries.zHeadPosition(:, sensorIndex); % this is probably unused
    
    % do NOT use prediction
    postureTimeSeries.uHeadPosition(:, index) = postureTimeSeries.xHeadPosition(:, sensorIndex);
    postureTimeSeries.vHeadPosition(:, index) = postureTimeSeries.yHeadPosition(:, sensorIndex) ...
                                          + cartesianVisualDrive_vel;
    postureTimeSeries.wHeadPosition(:, index) = postureTimeSeries.zHeadPosition(:, sensorIndex); % this is probably unused
end
function updateHeadOrientationSensorEstimates(index)
    orientationNoisePosition = parameters.masterNoise * noise.orientationNoisePosition(:, index);
    orientationNoiseVelocity = parameters.masterNoise * noise.orientationNoiseVelocity(:, index);
    orientationNoiseAcceleration = parameters.masterNoise * noise.orientationNoiseAcceleration(:, index);

    % sensor data of head variables
    postureTimeSeries.xHeadOrientation(:, index) = postureTimeSeries.headOrientation_p(:, index-1) + orientationNoisePosition;
    postureTimeSeries.yHeadOrientation(:, index) = postureTimeSeries.headOrientation_v(:, index-1) + orientationNoiseVelocity;
    postureTimeSeries.zHeadOrientation(:, index) = postureTimeSeries.headOrientation_a(:, index-1) + orientationNoiseAcceleration;
    
    % sensor delay
    delayTimeSteps = parameters.delay_central * parameters.frequency;
    sensorIndex = index - delayTimeSteps;
    if sensorIndex < 1
        sensorIndex = 1;
    end
    
    % estimate according to known delay
%     postureTimeSeries.uHeadOrientation(:, index) = postureTimeSeries.xHeadOrientation(:, sensorIndex) ...
%                                           + parameters.delay_central * postureTimeSeries.yHeadOrientation(:, sensorIndex);
%     postureTimeSeries.vHeadOrientation(:, index) = postureTimeSeries.yHeadOrientation(:, sensorIndex) ...
%                                           + parameters.delay_central * postureTimeSeries.zHeadOrientation(:, sensorIndex);
%     postureTimeSeries.wHeadOrientation(:, index) = postureTimeSeries.zHeadOrientation(:, sensorIndex); % this is probably unused
    
    % do NOT use prediction
    postureTimeSeries.uHeadOrientation(:, index) = postureTimeSeries.xHeadOrientation(:, sensorIndex);
    postureTimeSeries.vHeadOrientation(:, index) = postureTimeSeries.yHeadOrientation(:, sensorIndex);
    postureTimeSeries.wHeadOrientation(:, index) = postureTimeSeries.zHeadOrientation(:, sensorIndex);
    
end
function updateTrackingErrorSensorEstimates(index)
%     trackingErrorNoisePosition =  parameters.sigma_bp * sqrt(parameters.eulerStep) * parameters.masterNoise * randn(1, 1);
%     trackingErrorNoiseVelocity =  parameters.sigma_bv * sqrt(parameters.eulerStep) * parameters.masterNoise * randn(1, 1);
%     trackingErrorNoiseAcceleration =  parameters.sigma_ba * sqrt(parameters.eulerStep) * parameters.masterNoise * randn(1, 1);
    trackingErrorNoisePosition = parameters.masterNoise * noise.trackingErrorNoisePosition(:, index);
    trackingErrorNoiseVelocity = parameters.masterNoise * noise.trackingErrorNoiseVelocity(:, index);
    trackingErrorNoiseAcceleration = parameters.masterNoise * noise.trackingErrorNoiseAcceleration(:, index);

    % sensor data of head variables
    postureTimeSeries.xTrackingError(index) = postureTimeSeries.b(index-1) - postureTimeSeries.q(index-1) + trackingErrorNoisePosition;
    postureTimeSeries.yTrackingError(index) = postureTimeSeries.bDot(index-1) - postureTimeSeries.qDot(index-1) + trackingErrorNoiseVelocity;
    postureTimeSeries.zTrackingError(index) = postureTimeSeries.bTwoDot(index-1) - postureTimeSeries.qTwoDot(index-1) + trackingErrorNoiseAcceleration;
    
    % sensor delay
    delayTimeSteps = parameters.delay_central * parameters.frequency;
    sensorIndex = index - delayTimeSteps;
    if sensorIndex < 1
        sensorIndex = 1;
    end
    
    % estimate according to known delay
%     postureTimeSeries.uTrackingError(index) = postureTimeSeries.xTrackingError(sensorIndex) ...
%                                           + parameters.delay_central * postureTimeSeries.yTrackingError(sensorIndex);
%     postureTimeSeries.vTrackingError(index) = postureTimeSeries.yTrackingError(sensorIndex) ...
%                                           + parameters.delay_central * postureTimeSeries.zTrackingError(sensorIndex);
%     postureTimeSeries.wTrackingError(index) = postureTimeSeries.zTrackingError(sensorIndex); % this is probably unused
    
    % do NOT use prediction
    postureTimeSeries.uTrackingError(index) = postureTimeSeries.xTrackingError(sensorIndex);
    postureTimeSeries.vTrackingError(index) = postureTimeSeries.yTrackingError(sensorIndex);
    postureTimeSeries.wTrackingError(index) = postureTimeSeries.zTrackingError(sensorIndex);

end
function updateComEstimates(index)
%     comSensorNoisePosition = parameters.sigma_cp * sqrt(parameters.eulerStep) * parameters.masterNoise * randn(1, 1);
%     comSensorNoiseVelocity = 0;
%     comSensorNoiseAcceleration = 0;
    comSensorNoisePosition = parameters.masterNoise * noise.comSensorNoisePosition(:, index);
    comSensorNoiseVelocity = parameters.masterNoise * noise.comSensorNoiseVelocity(:, index);
    comSensorNoiseAcceleration = parameters.masterNoise * noise.comSensorNoiseAcceleration(:, index);
    
    % sensor data
    postureTimeSeries.xCom(index) = postureTimeSeries.com(2, index-1) + comSensorNoisePosition;

    % use sensor data with delay
%     delayTimeSteps = parameters.delay_poly * parameters.frequency;
    delayTimeSteps = parameters.delay_com * parameters.frequency;
    sensorIndex = index - delayTimeSteps - 1;
    if sensorIndex < 1
        sensorIndex = 1;
    end

    % CoP sensor data
%     if useContactDofs
% %         copSensorNoise =  parameters.sigma_cp * sqrt(parameters.eulerStep) * parameters.masterNoise * randn(1, 1);
%         copSensorNoise =  parameters.masterNoise * noise.copSensorNoise(index);
%         postureTimeSeries.xCop(index) = postureTimeSeries.cop(index-1) + copSensorNoise;
%         uComDot = -parameters.beta_cp*(postureTimeSeries.uCom(index-1) - postureTimeSeries.xCop(sensorIndex));
%         postureTimeSeries.uCom(index) = postureTimeSeries.uCom(index-1) + parameters.eulerStep * uComDot;
%     else
        uComDot = parameters.beta_mp*(postureTimeSeries.xCom(sensorIndex) - postureTimeSeries.uCom(:, index-1));
        postureTimeSeries.uCom(index) = postureTimeSeries.uCom(index-1) + parameters.eulerStep * uComDot;
%     end
    postureTimeSeries.yCom(index) = postureTimeSeries.comDot(2, index-1) + comSensorNoiseVelocity;
    postureTimeSeries.zCom(index) = postureTimeSeries.comTwoDot(2, index-1) + comSensorNoiseAcceleration;
    postureTimeSeries.vCom(index) = postureTimeSeries.yCom(sensorIndex);
    postureTimeSeries.wCom(index) = postureTimeSeries.zCom(sensorIndex);
end
function updateJointTorqueEstimates(index)
    % use sensor data with delay
    delayTimeSteps = parameters.delay_central * parameters.frequency;
    sensorIndex = index - delayTimeSteps - 1;
    if sensorIndex < 1
        sensorIndex = 1;
    end

%     torqueSensorNoise = parameters.sigma_tp * sqrt(parameters.eulerStep) * parameters.masterNoise * randn(parameters.bodyDofs, 1);
    torqueSensorNoise = parameters.masterNoise * noise.torqueSensorNoise(:, index);
    postureTimeSeries.xTorque(:, index-1) = postureTimeSeries.muscleTorque(:, index-1) + torqueSensorNoise;

    uTorqueDot = parameters.beta_pt*(postureTimeSeries.xTorque(:, sensorIndex) - postureTimeSeries.uTorque(:, index-1));
    postureTimeSeries.uTorque(:, index) = postureTimeSeries.uTorque(:, index-1) + parameters.eulerStep * uTorqueDot;
end

%% muscle dynamics

function updateMuscleActivation(index)
    motorNoise = parameters.masterNoise * noise.motorNoise(:, index);
    M = squeeze(postureTimeSeries.inertiaMatrix(:, :, index));

    % polysynaptic stretch reflex
    theta_est = postureTimeSeries.uJoint_poly(:, index);
    thetaDot_est = postureTimeSeries.vJoint_poly(:, index);
    lambda = postureTimeSeries.lambda(:, index-1);
    lambdaDot = postureTimeSeries.lambdaDot(:, index-1);
    
    % XXX turning off peripheral feedback
%     theta_est = postureTimeSeries.uJoint_poly(:, 1);
%     thetaDot_est = postureTimeSeries.vJoint_poly(:, 1);
    
    % tonic stretch reflex
    if useRelativeDamping
        agonistBase = parameters.muscle_alpha*(theta_est + parameters.muscle_cc - lambda + parameters.muscle_mu * (thetaDot_est - lambdaDot));
        antagonistBase = - parameters.muscle_alpha*(theta_est - parameters.muscle_cc - lambda + parameters.muscle_mu * (thetaDot_est - lambdaDot));
    else
        agonistBase = parameters.muscle_alpha*(theta_est + parameters.muscle_cc - lambda + parameters.muscle_mu * thetaDot_est);
        antagonistBase = - parameters.muscle_alpha*(theta_est - parameters.muscle_cc - lambda + parameters.muscle_mu * thetaDot_est);
    end
    
    agonistSlack = agonistBase<0;
    agonistBase(agonistSlack) = 0;
    agonistActivation = exp(agonistBase) - 1;
    antagonistSlack = antagonistBase<0;
    antagonistBase(antagonistSlack) = 0;
    antagonistActivation = exp(antagonistBase) - 1;
    if useBoundedActivation
        maximalActivation = 10;
        E0 = 3/4 * maximalActivation;
        b0 = log(E0 + 1);
        c2 = -b0 + maximalActivation/E0 - 1;
        c3 = maximalActivation;
        c1 = (b0 + c2)*(E0 - c3);
        agonistHyperbolic = c1 * (agonistBase + c2).^(-1) + c3;
        antagonistHyperbolic = c1 * (antagonistBase + c2).^(-1) + c3;
        agonistActivation(agonistBase > b0*ones(bodyDofs, 1)) = agonistHyperbolic(agonistBase > b0*ones(bodyDofs, 1));
        antagonistActivation(antagonistBase > b0*ones(bodyDofs, 1)) = antagonistHyperbolic(antagonistBase > b0*ones(bodyDofs, 1));
%         if max([agonistBase > b0*ones(bodyDofs, 1); antagonistBase > b0*ones(bodyDofs, 1)] == 1)
%             disp('approaching maximal torque');
%         end
    end
    totalDesiredChange = - agonistActivation + antagonistActivation;
    E = totalDesiredChange;
    
    % calculate change of activation by change of lambda
    d_agonistActivation_by_d_lambda = - parameters.muscle_alpha * exp(agonistBase);
    d_agonistActivation_by_d_lambda(agonistSlack) = 0;
    d_antagonistActivation_by_d_lambda = parameters.muscle_alpha * exp(antagonistBase);
    d_antagonistActivation_by_d_lambda(antagonistSlack) = 0;
    
    d_E_by_d_lambda = diag(- d_agonistActivation_by_d_lambda + d_antagonistActivation_by_d_lambda);
    
%     postureTimeSeries.muscleActivation(:, index) = E + motorNoise; % additive muscle noise
    postureTimeSeries.muscleActivation(:, index) = E .* (1 + motorNoise); % multiplicative muscle noise

    
    
end
function updateMuscleTorques(index)
    steadyStateTorque = parameters.muscleSetupMatrix * postureTimeSeries.muscleActivation(:, index);
    if useCalciumKinetics
        postureTimeSeries.muscleTorqueTwoDot(:, index) ...
            = 1/(parameters.tau_c^2) * (steadyStateTorque ...
                                         - 2 * parameters.tau_c * postureTimeSeries.muscleTorqueDot(:, index-1) ...
                                         - postureTimeSeries.muscleTorque(:, index-1));
        postureTimeSeries.muscleTorqueDot(:, index) = postureTimeSeries.muscleTorqueDot(:, index-1) + parameters.eulerStep * postureTimeSeries.muscleTorqueTwoDot(:, index);
        postureTimeSeries.muscleTorque(:, index) = postureTimeSeries.muscleTorque(:, index-1) + parameters.eulerStep * postureTimeSeries.muscleTorqueDot(:, index);
    else
        postureTimeSeries.muscleTorque(:, index) = steadyStateTorque;
    end
    
%     maximalTorque = 100;
%     for i_joint = 1 : bodyDofs
%         if abs(postureTimeSeries.muscleTorque(i_joint, index)) > maximalTorque
%             postureTimeSeries.muscleTorque(i_joint, index) = maximalTorque * sign(postureTimeSeries.muscleTorque(i_joint, index));
%         end
%     end
end

%% constraints

function constraintForces = calculateConstraintForces(index)
    if ~useContactDofs
        constraintForces = zeros(plant.mNumberOfJoints, 1);
        return;
    end
    
    
    % determine constraint DoFs
    if ~useKneeJointLimit && ~useContactDofs
%         c = [];
%         u = 1:plant.mBodyDofs;
        constraintForces = zeros(plant.mNumberOfJoints, 1);
        return;
    end

    % calculate accelerations from force platform
    accelerationFromForcePlatform = zeros(plant.mContactDofs, 1);
    if applyDisplacement  % vertical force plate perturbation force
        t = index * parameters.eulerStep;
        if displacementOnsetTime <= t && t <= displacementOnsetTime+displacementTime
            if displacementType == 0
                acceleration = rampDisplacementAcceleration(t-displacementOnsetTime, displacementTime, displacement);
            elseif displacementType == 2
                acceleration = sinusoidDisplacementAcceleration(t-displacementOnsetTime, displacementTime, displacement);
            else
                acceleration = 0;
            end
            accelerationFromForcePlatform(displacementDof) = -acceleration;
        end
    end
    
    % calculate case where knee joint limit constraint is not active
%     tau = postureTimeSeries.muscleTorque(:, index) + postureTimeSeries.limitTorque(:, index); % <-- this did not account for the new passive effects
    tau = postureTimeSeries.appliedTorque(:, index);
    
    theta = plant.mJointAngles(plant.mContactDofs+1:end);
    thetaDot = plant.mJointVelocities(plant.mContactDofs+1:end);
    if useContactDofs
        M_thetaGamma = plant.mInertiaMatrix(plant.mContactDofs+1:end, 1:plant.mContactDofs);
        M_theta = plant.mInertiaMatrix(plant.mContactDofs+1:end, plant.mContactDofs+1:end);
        C_thetaGamma = plant.mCoriolisMatrix(plant.mContactDofs+1:end, 1:plant.mContactDofs);
        C_theta = plant.mCoriolisMatrix(plant.mContactDofs+1:end, plant.mContactDofs+1:end);
        N_theta = plant.mGravitationalTorques(plant.mContactDofs+1:end);
        gammaDot = plant.mJointVelocities(1:plant.mContactDofs, 1);
        gammaTwoDot = accelerationFromForcePlatform;
        thetaTwoDot_unc = M_theta^(-1) ...
                              * (...
                                  tau  ...
                                  - M_thetaGamma * gammaTwoDot ...
                                  - C_thetaGamma * gammaDot ...
                                  - C_theta * plant.mJointVelocities(plant.mContactDofs+1:end) ...
                                  - N_theta ...
                               );
    else
        M = plant.mInertiaMatrix(plant.mContactDofs+1:end, plant.mContactDofs+1:end);
        C = plant.mCoriolisMatrix(plant.mContactDofs+1:end, plant.mContactDofs+1:end);
        N =  plant.mGravitationalTorques(plant.mContactDofs+1:end, 1);
        thetaTwoDot_unc = M^(-1) * (tau - C * plant.mJointVelocities(plant.mContactDofs+1:end) - N);
    end

    if ~useKneeJointLimit
        M_gamma = plant.mInertiaMatrix(1:plant.mContactDofs, 1:plant.mContactDofs);
        M_gammaTheta = plant.mInertiaMatrix(1:plant.mContactDofs, plant.mContactDofs+1:end);
        C_gamma = plant.mCoriolisMatrix(1:plant.mContactDofs, 1:plant.mContactDofs);
        C_gammaTheta = plant.mCoriolisMatrix(1:plant.mContactDofs, plant.mContactDofs+1:end);
        N_gamma = plant.mGravitationalTorques(1:plant.mContactDofs, 1);
        rho = M_gamma * gammaTwoDot ...
                  + M_gammaTheta * thetaTwoDot_unc ...
                  + C_gamma * gammaDot ...
                  + C_gammaTheta * thetaDot ...
                  + N_gamma;
        constraintForces = [rho; zeros(size(tau))];
        return;
    else
        M = plant.mInertiaMatrix;
        C = plant.mCoriolisMatrix;
        N =  plant.mGravitationalTorques;
        tau = [zeros(plant.mContactDofs, 1); tau];
        c = 1 : plant.mContactDofs;
        u = plant.mContactDofs+1 : plant.mNumberOfJoints;
        thetaKneeDot_unc = plant.mJointVelocities(plant.mContactDofs+2) + parameters.eulerStep * thetaTwoDot_unc(2);
        thetaKnee_unc = plant.mJointAngles(plant.mContactDofs+2) + parameters.eulerStep * thetaKneeDot_unc;
        % calculate acceleration from hitting the knee joint limit
        kneeJointLimitAcceleration = [];
        kneeJointLimitConstraintActive = false;
        if kneeJointConstraintVersion == 1
            % absolutely rigid constraint
            if thetaKnee_unc < parameters.kneeJointLimit
                kneeJointLimitAcceleration = - parameters.eulerStep^(-2)*theta(plant.mContactDofs+2) ...
                                             - parameters.eulerStep^(-1)*thetaDot(plant.mContactDofs+2);
                kneeJointLimitConstraintActive = true;
            end
        elseif kneeJointConstraintVersion == 2
            % slightly less rigid version that doesn't correct the initial breach within one time step
            if (plant.mJointAngles(plant.mContactDofs+2) < parameters.kneeJointLimit) && (thetaKneeDot_unc < 0)
                kneeJointLimitAcceleration = - parameters.eulerStep^(-1)*thetaDot(plant.mContactDofs+2);
                c = [1 : plant.mContactDofs plant.mContactDofs+2];
                u = [plant.mContactDofs+1 plant.mContactDofs+3 : plant.mNumberOfJoints];
            end
        elseif kneeJointConstraintVersion == 3
            % deforming constraint
            if (plant.mJointAngles(plant.mContactDofs+2) < parameters.kneeJointLimit) && (plant.mJointVelocities(plant.mContactDofs+2) < 0)
                factor = 1 - (thetaKnee_unc - parameters.kneeJointLimitRange) / (parameters.kneeJointLimit - parameters.kneeJointLimitRange);
                thetaKneeDot_con = thetaDot(plant.mContactDofs+2) * factor^2;
                kneeJointLimitAcceleration = - parameters.eulerStep^(-1)*thetaKneeDot_con;
                
                
                c = [1 : plant.mContactDofs plant.mContactDofs+2];
                u = [plant.mContactDofs+1 plant.mContactDofs+3 : plant.mNumberOfJoints];
                
            end
        end
        if kneeJointLimitConstraintActive
            c = [1 : plant.mContactDofs plant.mContactDofs+2];
            u = [plant.mContactDofs+1 plant.mContactDofs+3 : plant.mNumberOfJoints];
        end        
        
        % put together to form the acceleration in the constraint DoFs
        thetaTwoDot = zeros(plant.mNumberOfJoints, 1);
        thetaTwoDot(c) = [accelerationFromForcePlatform; kneeJointLimitAcceleration];

        % calculate accelerations at unconstrained joints
        thetaTwoDot(u) = M(u, u)^(-1) * (tau(u) - M(u, c)*thetaTwoDot(c) - C(u, c)*thetaDot(c) - C(u, u)*thetaDot(u) - N(u));
        constraintForces = zeros(plant.mNumberOfJoints, 1);
        constraintForces(c) = M(c, c)*thetaTwoDot(c) ...
                                  + M(c, u)*thetaTwoDot(u) ...
                                  + C(c, c)*thetaDot(c) ...
                                  + C(c, u)*thetaDot(u) ...
                                  + N(c) ...
                                  - tau(c);
        
    end
    


    postureTimeSeries.limitTorque(:, index) = constraintForces;
    
    
    
end

%% neural dynamics

function updateRho(index)
%     referenceNoise = parameters.sigma_r * sqrt(parameters.eulerStep) * parameters.masterNoise * randn(parameters.bodyDofs, 1);
    referenceNoise = parameters.masterNoise * noise.referenceNoise(:, index);
    postureTimeSeries.r_backCoupling(:, index) = calculateBackCoupling(index);
    postureTimeSeries.rhoDot(:, index) = postureTimeSeries.r_backCoupling(:, index);
    
    
    postureTimeSeries.rho(:, index) = postureTimeSeries.rho(:, index-1) + parameters.eulerStep * (postureTimeSeries.rhoDot(:, index) + referenceNoise);
end
function result = calculateBackCoupling(index)
    % calculate raw difference forcelet
    f = - parameters.alpha_bc * (postureTimeSeries.lambda(:, index-1) - postureTimeSeries.uJoint_poly(:, index-1));
    % project onto null space of CoM_hor
%     J_com_horizontal = internalModel.mComJacobian(2, :);
    J_com_horizontal = postureTimeSeries.J_com(:, index)';

    P = eye(plant.mBodyDofs) - J_com_horizontal' * J_com_horizontal * (1 / norm(J_com_horizontal)^2);
    
    result = P * f;
end
function updateLambda(index)
    % note: lambda is the equilibrium point of the tendon spring

    % calculate forcelets
    postureTimeSeries.f_jointPosition(:, index) = calculateJointPositionForcelet(index);% + calculateJointPositionForcelet_quad(index);
    postureTimeSeries.f_jointVelocity(:, index) = calculateJointVelocityForcelet(index);% + calculateJointVelocityForcelet_quad(index);
    postureTimeSeries.f_jointAcceleration(:, index) = calculateJointAccelerationForcelet(index);
    
    
    % turn joint forcelet off during voluntary movement
    currentTime = index * parameters.eulerStep;
    if doVoluntaryMovement && (voluntaryMovementOnsetTime<currentTime) && (currentTime<voluntaryMovementOnsetTime+voluntaryMovementTime)
        postureTimeSeries.f_jointPosition(:, index) = zeros(bodyDofs, 1);
        postureTimeSeries.f_jointVelocity(:, index) = zeros(bodyDofs, 1);
        postureTimeSeries.f_jointAcceleration(:, index) = zeros(bodyDofs, 1);
    end
    
    
%     postureTimeSeries.f_jointPosition(:, index) = calculateJointPositionForcelet_quad(index);
%     postureTimeSeries.f_jointVelocity(:, index) = calculateJointVelocityForcelet_quad(index);

%     postureTimeSeries.f_com(:, index) = calculateComForcelet_vtc(index);% + calculateComForcelet_vtc_quad(index);
    postureTimeSeries.f_com(:, index) = calculateComForcelet(index);
%     postureTimeSeries.f_torque(:, index) = calculateTorqueForcelet_prime(index);
    postureTimeSeries.f_head(:, index) = calculateHeadForcelet_position(index);
    postureTimeSeries.f_orientation(:, index) = calculateHeadForcelet_orientation(index);
%     postureTimeSeries.f_head(:, index) = calculateHeadForcelet_combined(index);
    postureTimeSeries.f_vol(:, index) = calculateVoluntaryMovementForcelet(index);
    
    % update neurally represented equilibrium point of the tendon spring (i.e. muscle length)
    neuralNoise = parameters.masterNoise * noise.neuralNoise(:, index);
    postureTimeSeries.lambdaDot(:, index) = postureTimeSeries.f_jointPosition(:, index) ...
                                            + postureTimeSeries.f_jointVelocity(:, index) ...
                                            + postureTimeSeries.f_jointAcceleration(:, index) ...
                                            + postureTimeSeries.f_com(:, index) ...
                                            + postureTimeSeries.f_head(:, index) ...
                                            + postureTimeSeries.f_vol(:, index) ...
                                            + postureTimeSeries.f_orientation(:, index) ...
                                            + neuralNoise ...
                                           ;
%     postureTimeSeries.lambdaDot(:, index) = 0;
    postureTimeSeries.lambda(:, index) = postureTimeSeries.lambda(:, index-1) + parameters.eulerStep*postureTimeSeries.lambdaDot(:, index);
    
    
%     postureTimeSeries.lambda(:, index) = postureTimeSeries.lambda(:, index-1);
end
function result = calculateJointPositionForcelet(index) 
    f = - parameters.alpha_jpl * (postureTimeSeries.uJoint_central(:, index) - postureTimeSeries.rho(:, index));
    J = eye(plant.mBodyDofs);
    transformationMatrix = taskToLambdaShiftTransformation(J);
%     if usePurelyLocalStrategy
%         transformationMatrix = J;
%     end
    result = parameters.alpha_j * transformationMatrix * f;

    % XXX only apply the joint feedback to the knee and hip joints
%     result(1) = 0;
end
function result = calculateJointPositionForcelet_quad(index)
    diff = postureTimeSeries.uJoint_poly(:, index) - postureTimeSeries.rho(:, index);
    g = diff.^2;
    J = 2 * diag(diff);
    f = - parameters.alpha_jpq * g;
    transformationMatrix = taskToLambdaShiftTransformation(J);
    if usePurelyLocalStrategy
        transformationMatrix = J;
    end
    result = parameters.alpha_j * transformationMatrix * f;
end
function result = calculateJointVelocityForcelet(index)
    f = - parameters.alpha_jvl * postureTimeSeries.vJoint_central(:, index);
    J = eye(plant.mBodyDofs);
    transformationMatrix = taskToLambdaShiftTransformation(J);
%     if usePurelyLocalStrategy
%         transformationMatrix = J;
%     end
    result = parameters.alpha_j * transformationMatrix * f;

    % XXX only apply the joint feedback to the knee and hip joints
%     result(1) = 0;
end
function result = calculateJointVelocityForcelet_quad(index)
    f = - parameters.alpha_jvq * postureTimeSeries.vJoint_poly(:, index).^2;
    
    J = 2 * diag(postureTimeSeries.vJoint_poly(:, index));
    transformationMatrix = taskToLambdaShiftTransformation(J);
    if usePurelyLocalStrategy
        transformationMatrix = J;
    end
    result = parameters.alpha_j * transformationMatrix * f;
end
function result = calculateJointAccelerationForcelet(index)
    f = - parameters.alpha_jal * postureTimeSeries.wJoint_central(:, index);
    J = eye(plant.mBodyDofs);
    transformationMatrix = taskToLambdaShiftTransformation(J);
%     if usePurelyLocalStrategy
%         transformationMatrix = J;
%     end
    result = parameters.alpha_j * transformationMatrix * f;
end
function result = calculateComForcelet(index)
    % behavioral forcelets
    f_pos = - parameters.alpha_cp * (postureTimeSeries.uCom(index) - comReference(2));
    f_vel = - parameters.alpha_cv * (postureTimeSeries.vCom(index));
    f_acc = - parameters.alpha_ca * (postureTimeSeries.wCom(index));
    postureTimeSeries.g_com(:, index) = f_pos + f_vel + f_acc;
    
    % transform to lambda shift
    J_com = postureTimeSeries.J_com(:, index)';
%     J_com(2:end) = 0;
    transformationMatrix = taskToLambdaShiftTransformation(J_com);

    
    
    result = parameters.alpha_c * transformationMatrix * (f_pos + f_vel + f_acc);
end
function result = calculateComForcelet_quad(index)
    
    % behavioral forcelets
    diff = postureTimeSeries.uCom(index) - comReference(2);
    g = diff^2;
    J = 2 * diff * postureTimeSeries.J_com(:, index)';
    gDotHat = J * postureTimeSeries.vJoint_poly(:, index);
    
    f_pos = - parameters.alpha_cp * g;
    f_vel = - parameters.alpha_cv * gDotHat;
    postureTimeSeries.g_com(:, index) = f_pos + f_vel;
    
    % transform to lambda shift
    transformationMatrix = taskToLambdaShiftTransformation(J);
    result = parameters.alpha_c * transformationMatrix * (f_pos + f_vel);
end
function result = calculateComForcelet_vtc(index)
    % calculate time to contact
    c_pos = .10;
    c_neg = 0; % arbitrary stability limits
    
    % use current values
%     c = postureTimeSeries.com(2, index-1);
%     v = postureTimeSeries.comDot(2, index-1);
%     a = postureTimeSeries.comTwoDot(2, index-1);
    
    % use estimates
    c = postureTimeSeries.uCom(index);
    v = postureTimeSeries.vCom(index);
%     a = postureTimeSeries.wCom(index);
    
    vtc_pos_inv = v / (c_pos-c);
    vtc_neg_inv = v / (c_neg-c);

    postureTimeSeries.vtc_pos(index) = (c_pos-c) / v;
    postureTimeSeries.vtc_neg(index) = (c_neg-c) / v;
    
    if postureTimeSeries.vtc_pos(index) > 0
        f_pos = - parameters.alpha_cpl * vtc_pos_inv;
    else
        f_pos = 0;
    end
    if postureTimeSeries.vtc_neg(index) > 0
        f_neg = parameters.alpha_cpl * vtc_neg_inv;
    else
        f_neg = 0;
    end
%     J = internalModel.mComJacobian(2, :);
    J = postureTimeSeries.J_com(:, index)';
    
    postureTimeSeries.g_com(:, index) = f_pos + f_neg;
    
    % transform to lambda shift
    transformationMatrix = taskToLambdaShiftTransformation(J);
    result = parameters.alpha_c * transformationMatrix * (f_pos + f_neg);
end
function result = calculateComForcelet_vtc_quad(index)
    % calculate time to contact
    c_pos = .10;
    c_neg = 0; % arbitrary stability limits
    
    % use current values
%     c = postureTimeSeries.com(2, index-1);
%     v = postureTimeSeries.comDot(2, index-1);
%     a = postureTimeSeries.comTwoDot(2, index-1);
    
    % use estimates
    c = postureTimeSeries.uCom(index);
    v = postureTimeSeries.vCom(index);
    
    vtc_pos_inv = v / (c_pos-c);
    vtc_neg_inv = v / (c_neg-c);

    postureTimeSeries.vtc_pos(index) = (c_pos-c) / v;
    postureTimeSeries.vtc_neg(index) = (c_neg-c) / v;
    
    if postureTimeSeries.vtc_pos(index) > 0
        f_pos = - parameters.alpha_cpq * vtc_pos_inv^2;
    else
        f_pos = 0;
    end
    if postureTimeSeries.vtc_neg(index) > 0
        f_neg = - parameters.alpha_cpq * vtc_neg_inv^2;
    else
        f_neg = 0;
    end
    J = 2 * v * postureTimeSeries.J_com(:, index)';

    postureTimeSeries.g_com(:, index) = f_pos + f_neg;
    
    % transform to lambda shift
    transformationMatrix = taskToLambdaShiftTransformation(J);
    result = parameters.alpha_c * transformationMatrix * (f_pos + f_neg);
end
function result = calculateHeadForcelet_position(index)
    % behavioral forcelets
    f_pos = - parameters.alpha_pp * (postureTimeSeries.uHeadPosition(2, index) - headReference_pos(2));
    f_vel = - parameters.alpha_pv * (postureTimeSeries.vHeadPosition(2, index));
    f_acc = - parameters.alpha_pa * (postureTimeSeries.wHeadPosition(2, index));
    postureTimeSeries.g_visual(:, index) = f_pos + f_vel + f_acc;

    % transform to lambda shift
    J_head = postureTimeSeries.J_head(:, index)';
    if useAnkleStrategy == 1
        J_head(2:end) = 0; % checking ankle strategy
%         J_head = [1 zeros(1, bodyDofs-1)]; % checking ankle strategy
    end
    transformationMatrix = taskToLambdaShiftTransformation(J_head);
    
    % XXX use com Jacobian instead
%     J_head = postureTimeSeries.J_head(:, index)'; 
%     J_com = postureTimeSeries.J_com(:, index)';
%     J_com_scaled = J_com * norm(J_head) / norm(J_com);
%     transformationMatrix = taskToLambdaShiftTransformation(J_com_scaled);
    
    result = parameters.alpha_p * transformationMatrix * (f_pos + f_vel + f_acc);
end
function result = calculateHeadForcelet_nonlinear(index)
    % behavioral forcelets
    f_vel = - parameters.alpha_pvq * (postureTimeSeries.vHeadPosition(2, index))^3;
    f_acc = - parameters.alpha_paq * (postureTimeSeries.wHeadPosition(2, index))^3;
    postureTimeSeries.g_visual(:, index) = f_vel + f_acc;

    % transform to lambda shift
    J_head = postureTimeSeries.J_head(:, index)';
    if useAnkleStrategy == 1
        J_head(2:end) = 0; % checking ankle strategy
        J_head = [1 zeros(1, bodyDofs-1)]; % checking ankle strategy
    end
    transformationMatrix = taskToLambdaShiftTransformation(J_head);
    
    % channel into CoM movement instead of head movement
%     J_com_horizontal = postureTimeSeries.J_com(:, index)';
%     transformationMatrix = taskToLambdaShiftTransformation(J_com_horizontal);
    
    result = parameters.alpha_p * transformationMatrix * (f_vel + f_acc);
end
function result = calculateHeadForcelet_orientation(index)
    % behavioral forcelets
    f_pos = - parameters.alpha_op * (postureTimeSeries.uHeadOrientation(index) - headReference_ori);
    f_vel = - parameters.alpha_ov * (postureTimeSeries.vHeadOrientation(index));
    f_acc = - parameters.alpha_oa * (postureTimeSeries.wHeadOrientation(index));

    % transform to lambda shift
    J_ori = ones(1, plant.mBodyDofs);
%     transformationMatrix = taskToLambdaShiftTransformation(J_ori);
%     result = parameters.alpha_o * transformationMatrix * (f_pos + f_vel + f_acc);
    
    % augment Jacobian to keep horizontal CoM position invariant
    J_com_horizontal = postureTimeSeries.J_com(:, index)';
%     J_head_horizontal = postureTimeSeries.J_head(:, index)';
    J_aug = [J_ori; J_com_horizontal];
    transformationMatrix = taskToLambdaShiftTransformation(J_aug);
    result = parameters.alpha_o * transformationMatrix * [f_pos + f_vel + f_acc; 0];
    
%     transformationMatrix = taskToLambdaShiftTransformation(J_ori);
%     result = parameters.alpha_o * transformationMatrix * [f_pos + f_vel + f_acc];
end
function result = calculateHeadForcelet_combined(index)
    % behavioral forcelets
    f_pp = - parameters.alpha_pp * (postureTimeSeries.uHeadPosition(2, index) - headReference_pos(2));
    f_pv = - parameters.alpha_pv * (postureTimeSeries.vHeadPosition(2, index));
    f_pa = - parameters.alpha_pa * (postureTimeSeries.wHeadPosition(2, index));
    f_op = - parameters.alpha_op * (postureTimeSeries.uHeadOrientation(index) - headReference_ori);
    f_ov = - parameters.alpha_ov * (postureTimeSeries.vHeadOrientation(index));
    f_oa = - parameters.alpha_oa * (postureTimeSeries.wHeadOrientation(index));
    f_p = f_pp + f_pv + f_pa;
    f_o = f_op + f_ov + f_oa;

    if index==2400
        blah=0;
    end
    
    % transform to lambda shift
    J_head = postureTimeSeries.J_head(:, index)';
    J_ori = ones(1, plant.mBodyDofs);
    J = [J_head; J_ori];
    transformationMatrix = taskToLambdaShiftTransformation(J);
    result = parameters.alpha_p * transformationMatrix * [f_p; f_o];
end
function result = calculateVoluntaryMovementForcelet(index)
    % behavioral forcelets
    if voluntaryMovementType==0
        direction_joint = [-1; 2; -1];
    elseif voluntaryMovementType==1
        direction_joint = [1; 1; 1];
    elseif voluntaryMovementType==2
        direction_joint = [-1; 0; 0];
    elseif voluntaryMovementType==3
        direction_joint = [1; 0; -2];
    end

    % transform to lambda shift
    J = eye(3, 3);
    transformationMatrix = taskToLambdaShiftTransformation(J);
    
    currentTime = index * parameters.eulerStep;
    velocity = sinusoidalDisplacementVelocity(currentTime, voluntaryMovementOnsetTime, voluntaryMovementOnsetTime+voluntaryMovementTime, voluntaryMovementMagnitude);
    f_vol = velocity * direction_joint;
                
    result = doVoluntaryMovement * transformationMatrix * f_vol;
end
function result = calculateTorqueForcelet(index)
    gravitationalTorque = plant.mGravitationalTorques;
    postureTimeSeries.gravitationalTorqueDot(:, index) = plant.gravitationJacobian * plant.mJointVelocities;

    % reduce the summed squares
    G = gravitationalTorque(1)^2 + gravitationalTorque(2)^2 + gravitationalTorque(3)^2;
%     diff = G - gravitationalTorqueSummedSquaresReference;
    diff = G;
    desiredSummedSquareChange = 0;
    if diff > 0
        desiredSummedSquareChange = - parameters.alpha_tp * diff;
    end
    
    % fully separated
%     J_G = plant.gravitationSummedSquaresJacobian;
%     postureTimeSeries.J_G(index, :) = J_G;
%     postureTimeSeries.J_G_pinv(:, index) = pinv(J_G);
    transformationMatrix = taskToLambdaShiftTransformation(J_G);
    result = parameters.alpha_t * transformationMatrix * desiredSummedSquareChange;
    
    % augment Jacobian to avoid affecting the horizontal CoM
%     J_com_horizontal = plant.mComJacobian(2, :);
%     transformationMatrix = taskToLambdaShiftTransformation([J_G; J_com_horizontal]);
%     result = parameters.alpha_t * transformationMatrix * [desiredSummedSquareChange; 0];
    
    
    
    
    
    

end
function result = calculateComForcelet_prime(index)
    % behavioral forcelets
    f_pos = - parameters.alpha_cp * (postureTimeSeries.uCom(index) - comReference(2));
    f_vel = - parameters.alpha_cv * (postureTimeSeries.vCom(index));

    % task Jacobian
    J_com_horizontal = plant.mComJacobian(2, :);
    
    % augment Jacobian to keep upper joints unaffected
    J_aug = [J_com_horizontal; [zeros(plant.mBodyDofs-1, 1) eye(plant.mBodyDofs-1)]];
    
    % transform to lambda shift
    transformationMatrix = taskToLambdaShiftTransformation(J_aug);
    result = parameters.alpha_c * transformationMatrix * [(f_pos + f_vel); zeros(plant.mBodyDofs-1, 1)];
end
function result = calculateTorqueForcelet_prime(index)
    % forcelet that reduces the summed squares of the joint torques except
    % the ankle torque, and leaves the CoM_hor invariant
    
    % use raw data
%     gravitationalTorqueEstimate = plant.mGravitationalTorques;
    postureTimeSeries.gravitationalTorqueDot(plant.mContactDofs+1:end, index) = internalModel.gravitationJacobian * postureTimeSeries.vJoint_central(:, index);

    % use sensor data
    gravitationalTorqueEstimate = postureTimeSeries.uTorque(:, index);

    % summed squares
    G_prime = sum(gravitationalTorqueEstimate(2:end).^2);
    dGPrime_by_dN = 2*gravitationalTorqueEstimate(2:end)';
    J_GPrime = dGPrime_by_dN * internalModel.gravitationJacobian(2:end, :);
%     postureTimeSeries.J_G(index, :) = J_GPrime;
    G_primeDot = J_GPrime * postureTimeSeries.vJoint_central(:, index);
    
    desiredChange = - parameters.alpha_tp * G_prime - parameters.alpha_tv * G_primeDot; 
    postureTimeSeries.g_torque(:, index) = desiredChange;
    
   
    % task Jacobian
%     J_G = plant.gravitationSummedSquaresJacobian;
%     transformationMatrix = taskToLambdaShiftTransformation(J_G);
%     result = parameters.alpha_t * transformationMatrix * desiredChange;
    
    % augment Jacobian to keep upper joints unaffected
%     J_aug = [J_GPrime; [1, zeros(1, plant.mBodyDofs-1)]];
%     transformationMatrix = taskToLambdaShiftTransformation(J_aug);
%     result = parameters.alpha_t * transformationMatrix * [desiredChange; 0];
    
    % augment Jacobian to CoM_hor unaffected
%     J_com_horizontal = internalModel.mComJacobian(2, :);
    J_com_horizontal = postureTimeSeries.J_com(:, index)';
    J_aug = [J_GPrime; J_com_horizontal];
    transformationMatrix = taskToLambdaShiftTransformation(J_aug);
    result = parameters.alpha_t * transformationMatrix * [desiredChange; 0];
    
    
    
    
    

end
function result = calculateTorqueForcelet_TwoPrime(index)
    % forcelet that reduces the squares of the joint torques except the ankle torque, and leaves the ankle joint invariant

    % use raw data
%     gravitationalTorqueEstimate = plant.mGravitationalTorques;
    postureTimeSeries.gravitationalTorqueDot(:, index) = plant.gravitationJacobian * plant.mJointVelocities;

    % use sensor data
    gravitationalTorqueEstimate = postureTimeSeries.uTorque(:, index);
    
    
%     gravitationalTorqueEstimate_delay = postureTimeSeries.uTorque(2:3, index)
%     gravitationalTorqueEstimate_noDelay = plant.mGravitationalTorques(2:3)
%     disp('-----------------------------------')

    % summed squares
    G_TwoPrime = gravitationalTorqueEstimate(2:end).^2;
    dGTwoPrime_by_dN = diag(2*plant.mGravitationalTorques(2:end));
    J_GTwoPrime = dGTwoPrime_by_dN * plant.gravitationJacobian(2:end, :);
%     G_primeDot = J_GPrime * plant.mJointVelocities;
    
    desiredChange = - parameters.alpha_tp * G_TwoPrime;% - parameters.alpha_tv * G_primeDot; 
    
   
    % task Jacobian
%     J_G = plant.gravitationSummedSquaresJacobian;
    
    % augment Jacobian to keep upper joints unaffected
    J_aug = [J_GTwoPrime; [1, zeros(1, plant.mBodyDofs-1)]];
    
    transformationMatrix = taskToLambdaShiftTransformation(J_aug);
    result = parameters.alpha_t * transformationMatrix * [desiredChange; 0];
    
    
    
    
    
    

end
function transformationMatrix = taskToLambdaShiftTransformation(taskJacobian)
    M = squeeze(postureTimeSeries.inertiaMatrix(:, :, index));
    
    transformationMatrix = d_E_by_d_lambda^(-1) ...
                            * parameters.muscleSetupMatrix^(-1) ...
                            * M ...
                            * pinv(taskJacobian);
%     if usePurelyLocalStrategy
%         transformationMatrix = pinv(taskJacobian);
%     end
    % XXX test out if keeping R changes anything
    if usePurelyLocalStrategy
        transformationMatrix = d_E_by_d_lambda^(-1) ...
                                * pinv(taskJacobian);
    end
end

%% GUI

function plotVisibilityCheckBoxes = createControlFigure()
    % create GUI
    controlFig = figure('units', 'normalized', 'Position', [0.7, 0.65, 0.3, 0.25], ...
                    'Name', 'Kommandozentralissimo', ...
                    'MenuBar', 'none', 'NumberTitle', 'off');
	width = 1/5;
    uicontrol(controlFig, 'Style', 'Pushbutton',...
                                      'Units', 'Norm',...
                                      'String', 'CoM',...
                                      'Callback', @showComPlots,...
                                      'Position', [ .0, 0/4, width, 1/4 ]);
    uicontrol(controlFig, 'Style', 'Pushbutton',...
                                      'Units', 'Norm',...
                                      'String', 'ori',...
                                      'Callback', @showOrientationPlots,...
                                      'Position', [ .0, 1/4, width, 1/4 ]);
    uicontrol(controlFig, 'Style', 'Pushbutton',...
                                      'Units', 'Norm',...
                                      'String', 'm. equiv.',...
                                      'Callback', @motorEquivalence,...
                                      'Position', [ .0, 2/4, width, 1/4 ]);
    uicontrol(controlFig, 'Style', 'Pushbutton',...
                                      'Units', 'Norm',...
                                      'String', 'ucm',...
                                      'Callback', @ucm,...
                                      'Position', [ .0, 3/4, width, 1/4 ]);
    uicontrol(controlFig, 'Style', 'Pushbutton',...
                                      'Units', 'Norm',...
                                      'String', 'quit',...
                                      'Callback', @quitProgram,...
                                      'Position', [ width, 0/4, width, 1/4 ]);
    uicontrol(controlFig, 'Style', 'Pushbutton',...
                                      'Units', 'Norm',...
                                      'String', 'save',...
                                      'Callback', @saveData,...
                                      'Position', [ width, 1/4, width, 1/4 ]);
    uicontrol(controlFig, 'Style', 'Pushbutton',...
                                      'Units', 'Norm',...
                                      'String', 'video',...
                                      'Callback', @showVideo,...
                                      'Position', [ width, 2/4, width, 1/4 ]);
    uicontrol(controlFig, 'Style', 'Pushbutton',...
                                      'Units', 'Norm',...
                                      'String', 'more plots',...
                                      'Callback', @morePlots,...
                                      'Position', [ width, 3/4, width, 1/4 ]);
    plotVisibilityCheckBoxes = zeros(14, 1);
	plotVisibilityCheckBoxes(6) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', '\theta_6',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 2*width, 0/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(5) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', '\theta_5',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 2*width, 1/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(4) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', '\theta_4',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 2*width, 2/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(3) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', '\theta_3',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 2*width, 3/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(2) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', '\theta_2',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 2*width, 4/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(1) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', '\theta_1',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 2*width, 5/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(7) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', '\lambda',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 3*width, 5/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(8) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', 'sensor',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 3*width, 4/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(14) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', 'human',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 3*width, 3/6, width, 1/6 ]);
                                  
	plotVisibilityCheckBoxes(9) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', 'f-total',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 4*width, 5/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(10) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', 'f-joint_v',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 4*width, 4/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(11) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', 'f-com',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 4*width, 3/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(12) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', 'f-head',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 4*width, 2/6, width, 1/6 ]);
	plotVisibilityCheckBoxes(13) = uicontrol(controlFig, 'Style', 'checkbox',...
                                      'Units', 'Norm',...
                                      'String', 'f_ori',...
                                      'Callback', @updatePlotVisibility,...
                                      'Position', [ 4*width, 1/6, width, 1/6 ]);
                                  
                                  
    for i = 1:parameters.bodyDofs
        if timeSeriesFigure.mIsDrawingJoint(i)
            set(plotVisibilityCheckBoxes(i), 'Value', get(plotVisibilityCheckBoxes(i), 'Max'));
        else
            set(plotVisibilityCheckBoxes(i), 'Value', get(plotVisibilityCheckBoxes(i), 'Min'));
        end
    end
    if timeSeriesFigure.mIsDrawingLambda
        set(plotVisibilityCheckBoxes(7), 'Value', get(plotVisibilityCheckBoxes(7), 'Max'));
    else                                  
        set(plotVisibilityCheckBoxes(7), 'Value', get(plotVisibilityCheckBoxes(7), 'Min'));
    end
    if timeSeriesFigure.mIsDrawingSensor
        set(plotVisibilityCheckBoxes(8), 'Value', get(plotVisibilityCheckBoxes(8), 'Max'));
    else                                  
        set(plotVisibilityCheckBoxes(8), 'Value', get(plotVisibilityCheckBoxes(8), 'Min'));
    end
    if timeSeriesFigure.isDrawingFTotal
        set(plotVisibilityCheckBoxes(9), 'Value', get(plotVisibilityCheckBoxes(9), 'Max'));
    else                                  
        set(plotVisibilityCheckBoxes(9), 'Value', get(plotVisibilityCheckBoxes(9), 'Min'));
    end
    if timeSeriesFigure.isDrawingFJointVelocity
        set(plotVisibilityCheckBoxes(10), 'Value', get(plotVisibilityCheckBoxes(10), 'Max'));
    else                                  
        set(plotVisibilityCheckBoxes(10), 'Value', get(plotVisibilityCheckBoxes(10), 'Min'));
    end
    if timeSeriesFigure.isDrawingFCom
        set(plotVisibilityCheckBoxes(11), 'Value', get(plotVisibilityCheckBoxes(11), 'Max'));
    else                                  
        set(plotVisibilityCheckBoxes(11), 'Value', get(plotVisibilityCheckBoxes(11), 'Min'));
    end
    if timeSeriesFigure.isDrawingFHead
        set(plotVisibilityCheckBoxes(12), 'Value', get(plotVisibilityCheckBoxes(12), 'Max'));
    else                                  
        set(plotVisibilityCheckBoxes(12), 'Value', get(plotVisibilityCheckBoxes(12), 'Min'));
    end
    if timeSeriesFigure.isDrawingFOrientation
        set(plotVisibilityCheckBoxes(13), 'Value', get(plotVisibilityCheckBoxes(13), 'Max'));
    else                                  
        set(plotVisibilityCheckBoxes(13), 'Value', get(plotVisibilityCheckBoxes(13), 'Min'));
    end
    if timeSeriesFigure.isDrawingHuman
        set(plotVisibilityCheckBoxes(14), 'Value', get(plotVisibilityCheckBoxes(14), 'Max'));
    else                                  
        set(plotVisibilityCheckBoxes(14), 'Value', get(plotVisibilityCheckBoxes(14), 'Min'));
    end

    
end
function updatePlots(index)
    if useSecondaryTask
        stickFigure.update ...
        ( ...
            plant, ...
            postureTimeSeries.q(index), ...
            postureTimeSeries.b(index), ...
            secondaryTask_floor, ...
            secondaryTask_ceiling ...
        );
    else
        stickFigure.update(plant);
    end
    timeSeriesFigure.update(postureTimeSeries, index);

    drawnow;
end
function updatePlotVisibility(hObject, eventdata) %#ok<INUSD>
    for i = 1:parameters.bodyDofs
        if (get(plotVisibilityCheckBoxes(i),'Value') == get(plotVisibilityCheckBoxes(i),'Max'))
            timeSeriesFigure.mIsDrawingJoint(i) = 1;
        else
            timeSeriesFigure.mIsDrawingJoint(i) = 0;
        end
    end
    if (get(plotVisibilityCheckBoxes(7),'Value') == get(plotVisibilityCheckBoxes(7),'Max'))
        timeSeriesFigure.mIsDrawingLambda = 1;
    else
        timeSeriesFigure.mIsDrawingLambda = 0;
    end
    if (get(plotVisibilityCheckBoxes(8),'Value') == get(plotVisibilityCheckBoxes(8),'Max'))
        timeSeriesFigure.mIsDrawingSensor = 1;
    else
        timeSeriesFigure.mIsDrawingSensor = 0;
    end
    if (get(plotVisibilityCheckBoxes(9),'Value') == get(plotVisibilityCheckBoxes(9),'Max'))
        timeSeriesFigure.isDrawingFTotal = 1;
    else
        timeSeriesFigure.isDrawingFTotal = 0;
    end
    if (get(plotVisibilityCheckBoxes(10),'Value') == get(plotVisibilityCheckBoxes(10),'Max'))
        timeSeriesFigure.isDrawingFJointVelocity = 1;
    else
        timeSeriesFigure.isDrawingFJointVelocity = 0;
    end
    if (get(plotVisibilityCheckBoxes(11),'Value') == get(plotVisibilityCheckBoxes(11),'Max'))
        timeSeriesFigure.isDrawingFCom = 1;
    else
        timeSeriesFigure.isDrawingFCom = 0;
    end
    if (get(plotVisibilityCheckBoxes(12),'Value') == get(plotVisibilityCheckBoxes(12),'Max'))
        timeSeriesFigure.isDrawingFHead = 1;
    else
        timeSeriesFigure.isDrawingFHead = 0;
    end
    if (get(plotVisibilityCheckBoxes(13),'Value') == get(plotVisibilityCheckBoxes(13),'Max'))
        timeSeriesFigure.isDrawingFOrientation = 1;
    else
        timeSeriesFigure.isDrawingFOrientation = 0;
    end
    if (get(plotVisibilityCheckBoxes(14),'Value') == get(plotVisibilityCheckBoxes(14),'Max'))
        timeSeriesFigure.isDrawingHuman = 1;
    else
        timeSeriesFigure.isDrawingHuman = 0;
    end
    
    updatePlots(index);
end

%% program calls

function impulse(hObject, eventdata) %#ok<DEFNU,INUSD>
% calculates the total absolute impulse the muscles generated 

    I = sum(abs(externalTorque), 2) * parameters.eulerStep %#ok<NOPRT>
    totalImpulse = sum(I) %#ok<NASGU,NOPRT>

    L = sum(abs(lambdaDot), 2) * parameters.eulerStep %#ok<NOPRT>
    totalLambdaChange = sum(L) %#ok<NASGU,NOPRT>

end
function ucm(hObject, eventdata) %#ok<INUSD>
    thetaMean = mean(postureTimeSeries.theta(:, 2:end), 2);
    
    plant.mJointAngles = thetaMean;
    plant = plant.updateInternals();
    J_y = plant.mEndEffectorJacobian(2, plant.mContactDofs+1:end);
%     J_ori = ones(1, plant.mBodyDofs);
    J_com = plant.mComJacobian(2, :);
    
    theta = postureTimeSeries.theta(plant.mContactDofs+1:end, 2:end);
    [VPara_y, VPerp_y, ~, kPara_y, kPerp_y] = ucmAnalysis(theta, J_y);
%     [VPara_ori, VPerp_ori, ~, kPara_ori, kPerp_ori] = ucmAnalysis(theta, J_ori);
    [VPara_com, VPerp_com, ~, kPara_com, kPerp_com] = ucmAnalysis(theta, J_com);
    
    % visualize
    figure('Position', [ 1200, 600, 300, 300 ], ...
                    'Name', 'UCM - head position (y)', ...
                    'NumberTitle', 'off');            

    axes('Position', [ 0.1 0.1 0.8 0.8 ]);
    hold on;
    bar(1, VPara_y/kPara_y, 'g');
    bar(2, VPerp_y/kPerp_y, 'r');
    set(gca,'xtick',[1,2]);
    set(gca,'XTickLabel',{'GEV';'NGEV'});
    hold off;

%     figure('Position', [ 900, 600, 300, 300 ], ...
%                     'Name', 'UCM - head orientation', ...
%                     'NumberTitle', 'off');            
% 
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]);
%     hold on;
%     bar(1, VPara_ori/kPara_ori, 'g');
%     bar(2, VPerp_ori/kPerp_ori, 'r');
%     set(gca,'xtick',[1,2]);
%     set(gca,'XTickLabel',{'GEV';'NGEV'});
%     hold off;

    figure('Position', [ 900, 600, 300, 300 ], ...
                    'Name', 'UCM - CoM', ...
                    'NumberTitle', 'off');            

    axes('Position', [ 0.1 0.1 0.8 0.8 ]);
    hold on;
    bar(1, VPara_com/kPara_com, 'g');
    bar(2, VPerp_com/kPerp_com, 'r');
    set(gca,'xtick',[1,2]);
    set(gca,'XTickLabel',{'GEV';'NGEV'});
    hold off;
end
function motorEquivalence(hObject, eventdata) %#ok<INUSD>
%     time = postureTimeSeries.time;
    % use the initial state as the reference
    internalModel.mJointAngles = postureTimeSeries.theta(:, 1);
    internalModel = internalModel.updateInternals();
%     J_y = internalModel.mEndEffectorJacobian(2, :);
    J_com = internalModel.mComJacobian(2, :);
    
    % decomposition and projection matrices
    k_perp = size(J_com, 1);
    k_para = size(J_com, 2) - k_perp;
    [~, ~, V] = svd(J_com);
    E = V(k_perp+1:end, :);                              % E gives the base vectors of the null space
    H = E'*E;                                       % projection matrix to the null space
    
    % calculate projections of difference vectors
    theta = postureTimeSeries.theta(plant.mContactDofs+1:end, :);
    thetaInit = parameters.jointReference;%theta(:, 1);
    thetaDiff = zeros(size(theta));
    for j = 1:size(theta, 2)
        thetaDiff(:, j) = theta(:, j) - thetaInit;
    end
    theta_par = H*thetaDiff;
    theta_perp = thetaDiff - theta_par;
    
    % calculate length of projections
    D_par = zeros(1, length(theta));
    D_perp = zeros(1, length(theta));
    for l = 1:length(theta)
        D_par(l) = 1 / k_para * norm(theta_par(:, l));
        D_perp(l) = 1 / k_perp * norm(theta_perp(:, l));
    end
    
    
    % plot results
    figure('Position', [ 1200, 600, 600, 300 ], ...
                    'Name', 'CoM motor equivalence', ...
                    'NumberTitle', 'off');            
    axes('Position', [ 0.1 0.1 0.8 0.8 ]);
    hold on;
    
    plot(postureTimeSeries.time(2:index), rad2deg(D_par(2:index) * 1/2^(0.5)), 'g', 'linewidth', 2, 'DisplayName', 'par');
    plot(postureTimeSeries.time(2:index), rad2deg(D_perp(2:index)), 'r', 'linewidth', 2, 'DisplayName', 'perp');
%     plot(postureTimeSeries.time(2:index), 0.0001 * (postureTimeSeries.headPosition_p(2, 2:index) - postureTimeSeries.headPosition_p(2, 2)), 'b', 'linewidth', 2, 'DisplayName', 'head');

    legend('toggle');
    plot([postureTimeSeries.time(2), postureTimeSeries.time(index)], [0, 0], 'k:', 'linewidth', 1);


%     % collect bins
%     binLength = 1;
%     numberOfBins = time(end) / binLength;
%     binSteps = round(binLength / parameters.eulerStep);
%     B_par = zeros(1, numberOfBins);
%     B_perp = zeros(1, numberOfBins);
%     S = zeros(1, numberOfBins);
%     for i = 1 : numberOfBins
%         first = 1 + binSteps * (i-1);
%         last = binSteps * i;
%         B_par(i) = mean(D_par(first:last));
%         B_perp(i) = mean(D_perp(first:last));
%         S(i) = B_par(i) / B_perp(i);
%     end
%     
%     figure('Position', [ 600, 600, 600, 300 ], ...
%                     'Name', 'com / cop', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]);
%     hold on;
%     
%     
%     plot(S, 'b', 'linewidth', 2, 'DisplayName', 'S');
%     legend('toggle');

end
function showComPlots(hObject, eventdata) %#ok<INUSD>
    fullTimeEpisodeStart = 10;
    fullTimeEpisodeLength = postureTimeSeries.T;
    Fs = 120;
    com = [];
    load com.mat
    com_human = com{2, 2, 1};
    episodeStart_human = fullTimeEpisodeStart * Fs + 1;
    episodeEnd_human = min(episodeStart_human + fullTimeEpisodeLength * Fs - 1, size(com_human, 2));
    com_human = com_human(episodeStart_human : episodeEnd_human) - mean(com_human(episodeStart_human : episodeEnd_human)) + postureTimeSeries.com(2, 1);
    humanTimeData = 1/Fs : 1/Fs : (episodeEnd_human - episodeStart_human + 1)/Fs;
    figure('Position', [ 1200, 600, 600, 300 ], ...
                    'Name', 'CoM estimates', ...
                    'NumberTitle', 'off');            
    axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
    plot(humanTimeData, com_human, '-k', 'linewidth', 2, 'displayname', 'human');
    plot(postureTimeSeries.time(2:index), postureTimeSeries.com(2, 2:index), '-r', 'linewidth', 2, 'displayname', 'com');
    plot(postureTimeSeries.time(2:index), postureTimeSeries.xCom(:, 2:index), '-.r', 'linewidth', 1, 'displayname', 'xCom');
    plot(postureTimeSeries.time(2:index), postureTimeSeries.uCom(:, 2:index), '--r', 'linewidth', 1, 'displayname', 'uCom');
%     plot(postureTimeSeries.time(2:index), noise.comSensorNoisePosition(:, 2:index), '-m', 'linewidth', 2, 'displayname', '\xi_p');

%     plot(postureTimeSeries.time(2:index), postureTimeSeries.comDot(2, 2:index), '-g', 'linewidth', 2, 'displayname', 'comDot');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.yCom(:, 2:index), '--g', 'linewidth', 1, 'displayname', 'vCom');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.vCom(:, 2:index), ':g', 'linewidth', 1, 'displayname', 'yCom');
% 
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.comTwoDot(2, 2:index), '-b', 'linewidth', 2, 'displayname', 'comTwoDot');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.wCom(:, 2:index), '--b', 'linewidth', 1, 'displayname', 'wCom');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.zCom(:, 2:index), ':b', 'linewidth', 1, 'displayname', 'zCom');

%     plot(postureTimeSeries.time(2:index), noise.comSensorNoiseVelocity(:, 2:index), '-c', 'linewidth', 2, 'displayname', '\xi_v');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.yCom(:, 2:index), '-.g', 'linewidth', 1, 'displayname', 'yCom');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.cop(2:index), '-b', 'linewidth', 2, 'displayname', 'cop');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.xCop(2:index), '-.b', 'linewidth', 2, );
    legend('toggle');
    
end
function showOrientationPlots(hObject, eventdata) %#ok<INUSD>
    figure('Position', [ 1200, 600, 600, 300 ], ...
                    'Name', 'head orientation', ...
                    'NumberTitle', 'off');            
    axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
    plot([postureTimeSeries.time(2), postureTimeSeries.time(end)], [headReference_ori headReference_ori], '--r', 'linewidth', 1, 'displayname', 'ref');
    plot(postureTimeSeries.time(2:index), postureTimeSeries.headOrientation_p(2:index), '-r', 'linewidth', 2, 'displayname', 'p_{ori}');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.headOrientation_v(2:index), '-g', 'linewidth', 2, 'displayname', 'v_{ori}');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.headOrientation_a(2:index), '-b', 'linewidth', 2, 'displayname', 'a_{ori}');
    legend('toggle');
    
end
function morePlots(hObject, eventdata) %#ok<INUSD>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% psd
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     frequencies = []; psdData_human = []; psdMean_human = []; frequencies_human = [];
%     load /Users/reimajbi/Neuro/Posture/Matlab_data2006/subject_data/PSD_3DoF.mat
%     angles = postureTimeSeries.theta(plant.mContactDofs+1:end, 2:end);
%     angles = angles - repmat(mean(angles, 2), 1, size(angles, 2));
%     windowLength = 20;
%     nfft = postureTimeSeries.frequency*windowLength;
%     h = spectrum.welch('Hamming', nfft, 50);
%     psdData_model = cell(plant.mBodyDofs, 1);
%     for i_joint = 1 : plant.mBodyDofs
%         psdData_model{i_joint} = psd(h, angles(i_joint, :), 'NFFT', nfft, 'Fs', postureTimeSeries.frequency);
%     end
%     dataPoints = 3:150;
%     figure
%     for i_joint = 1 : bodyDofs
%         % model
%         loglog ...
%         ( ...
%             psdData_model{i_joint}.Frequencies(dataPoints), ...
%             psdData_model{i_joint}.data(dataPoints, 1), ...
%             'o--', ...
%             'color', [1, (i_joint-1)/plant.mBodyDofs, 0],...
%             'linewidth', 1, ...
%             'DisplayName', ['model - joint ', num2str(i_joint)] ...
%         )
%         hold on;
%         % human
%         loglog ...
%         ( ...
%             frequencies(dataPoints), ...
%             psdMean_human(i_joint, dataPoints, 1), ...
%             'x-', ...
%             'color', [1, (i_joint-1)/plant.mBodyDofs, 0],...
%             'linewidth', 1, ...
%             'DisplayName', ['human - joint ', num2str(i_joint)] ...
%         )
%     end
%     set(gca, 'xlim', [(psdData_model{i_joint}.Frequencies(dataPoints(1)))-0.01, psdData_model{i_joint}.Frequencies(dataPoints(end))]);
%     ylabel('PSD (deg^2/Hz)')
%     xlabel('Frequency [Hz]')
%     legend('toggle')
%     return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% secondary task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'tracking', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.b(2:index), 'r', 'linewidth', 2, 'DisplayName', 'b');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.bDot(2:index), 'g', 'linewidth', 2, 'DisplayName', 'bDot');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.bTwoDot(2:index), 'b', 'linewidth', 2, 'DisplayName', 'bTwoDot');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.q(2:index), 'r--', 'linewidth', 2, 'DisplayName', 'q');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.qDot(2:index), 'g--', 'linewidth', 2, 'DisplayName', 'qDot');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.qTwoDot(2:index), 'b--', 'linewidth', 2, 'DisplayName', 'qTwoDot');
%     legend('toggle');
% 
%     % secondary task, estimations
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'tracking error', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.b(2:index)-postureTimeSeries.q(2:index), 'r', 'linewidth', 2, 'DisplayName', 'b-q');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.bDot(2:index)-postureTimeSeries.qDot(2:index), 'g', 'linewidth', 2, 'DisplayName', 'bDot-qDot');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.bTwoDot(2:index)-postureTimeSeries.qTwoDot(2:index), 'b', 'linewidth', 2, 'DisplayName', 'bTwoDot-qTwoDot');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.xTrackingError(2:index), 'r', 'linewidth', 1, 'DisplayName', 'xTrackingError');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.yTrackingError(2:index), 'g', 'linewidth', 1, 'DisplayName', 'yTrackingError');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.zTrackingError(2:index), 'b', 'linewidth', 1, 'DisplayName', 'zTrackingError');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.uTrackingError(2:index), 'r--', 'linewidth', 1, 'DisplayName', 'uTrackingError');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.vTrackingError(2:index), 'g--', 'linewidth', 1, 'DisplayName', 'vTrackingError');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.wTrackingError(2:index), 'b--', 'linewidth', 1, 'DisplayName', 'wTrackingError');
%     legend('toggle');









%     % joint angle comparison
%     jointAngles = [];
%     Fs = 120;
%     time_human = 1/Fs : 1/Fs : T;
%     load /Users/reimajbi/Neuro/Posture/Matlab_data2006/subject_data/jointAngles_3DoF.mat
%     humanJointAngles = jointAngles{2, 2, 1}(:, 1:length(time_human));
%     humanJointAngles = humanJointAngles + repmat(postureTimeSeries.theta(plant.mContactDofs+1:end, 2) - humanJointAngles(:, 1), 1, length(time_human));
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'joint angle comparison', ...
%                     'NumberTitle', 'off');
% 	axes; hold on;
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.theta(plant.mContactDofs+1:end, 2:index), '-', 'linewidth', 2);
%     plot(time_human, humanJointAngles, '-.', 'linewidth', 2);
%     legend('ankle model', 'knee model', 'hip model', 'ankle human', 'knee human', 'hip human')

    % forcelet data
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'forcelet', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.f_jointPosition(:, 2:index), 'r', 'linewidth', 2, 'displayname', 'joint position');
% %     plot(postureTimeSeries.time(2:index), postureTimeSeries.g_com(2:index), '-g', 'linewidth', 2, 'displayname', 'com');
% %     plot(postureTimeSeries.time(2:index), postureTimeSeries.g_torque(2:index), '-b', 'linewidth', 2, 'displayname', 'torque');
%     legend('toggle');
%     return

%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'joint position estimates', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.theta(:, 2:index), '-', 'linewidth', 2);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.lambda(:, 2:index), '-.', 'linewidth', 2);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.rho(:, 2:index), '--', 'linewidth', 2);

    % sensor data
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'joint position estimates', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.theta(:, 2:index), '-', 'linewidth', 2);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.xJoint(:, 2:index), '-', 'linewidth', 1);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.uJoint_poly(:, 2:index), '-.', 'linewidth', 1);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.uJoint_central(:, 2:index), '--', 'linewidth', 1);
% 
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'joint velocity estimates', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.thetaDot(:, 2:index), '-', 'linewidth', 2);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.yJoint(:, 2:index), '-.', 'linewidth', 1);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.vJoint_poly(:, 2:index), '-.', 'linewidth', 1);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.vJoint_central(:, 2:index), '--', 'linewidth', 1);
% 
    
%     
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'virtual time to contact', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.vtc_pos(2:index).^(-1), '-g', 'linewidth', 2);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.vtc_neg(2:index).^(-1), '-b', 'linewidth', 2);
%     legend('vtc_{pos}^{-1}', 'vtc_{neg}^{-1}');
%     
    
    % orientation stuff
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'orientations', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.headOrientation_p(2:index), 'r', 'linewidth', 1, 'DisplayName', 'pos');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.uHeadOrientation(2:index), 'r--', 'linewidth', 1, 'DisplayName', '\hat pos');
%     legend('toggle');
    
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'orientations', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.headOrientation_v(2:index), 'g', 'linewidth', 1, 'DisplayName', 'vel');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.vHeadOrientation(2:index), 'g--', 'linewidth', 1, 'DisplayName', 'vel_e');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.headOrientation_a(2:index), 'b', 'linewidth', 1, 'DisplayName', 'acc');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.wHeadOrientation(2:index), 'b--', 'linewidth', 1, 'DisplayName', 'acc_e');
%     legend('toggle');
    
%     % summed squares of gravitational torques
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'torques', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     plot(postureTimeSeries.time(2:index), sum(postureTimeSeries.gravitationalTorque(2:3, 2:index).^2), 'm', 'linewidth', 2, 'DisplayName', 'sum of squared torques');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.gravitationalTorque(2, 2:index).^2, 'r', 'linewidth', 2, 'DisplayName', 'knee torque squared');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.gravitationalTorque(3, 2:index).^2, 'g', 'linewidth', 2, 'DisplayName', 'hip torque squared');
%     legend('toggle');
    
    
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'joint torque estimates', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.muscleTorque(:, 2:index), '--', 'linewidth', 2);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.xTorque(:, 2:index), '-.', 'linewidth', 1);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.uTorque(:, 2:index), '-', 'linewidth', 2);
    
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'joint torque forcelet', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
% %     plot(postureTimeSeries.time(2:index), postureTimeSeries.muscleTorque(:, 2:index), '--', 'linewidth', 2);
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.f_pt(:, 2:index), '-', 'linewidth', 2);
    
%     figure('Position', [ 1200, 600, 600, 300 ], ...
%                     'Name', 'com / cop', ...
%                     'NumberTitle', 'off');            
%     axes('Position', [ 0.1 0.1 0.8 0.8 ]); hold on;
%     
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.groundReactionForces(1, 2:index), 'r', 'linewidth', 2, 'DisplayName', 'f_y');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.groundReactionForces(2, 2:index), 'g', 'linewidth', 2, 'DisplayName', 'f_z');
%     plot(postureTimeSeries.time(2:index), postureTimeSeries.groundReactionForces(3, 2:index), 'b', 'linewidth', 2, 'DisplayName', 'M_x');
%     legend('toggle');
%     plot([postureTimeSeries.time(2), postureTimeSeries.time(index)], [0, 0], 'k:', 'linewidth', 1);
    
end
function showVideo(hObject, eventdata) %#ok<INUSD>
    % create vector of frames to show
    fps = 20;
    interval = (parameters.eulerStep * fps)^(-1);
    
    frames = 2 : interval : length(postureTimeSeries.time);
    progressBar = waitbar(0, 'Showing Fideo', 'Position', [ 1200, 1600, 300, 50 ]);

    for j = frames
        waitbar(j / length(postureTimeSeries.time), progressBar, 'shoving fideo')
        % update geometric transformations
        plant.mJointAngles = postureTimeSeries.theta(:, j);
        plant = plant.updateInternals;
        if useSecondaryTask
            stickFigure.update ...
            ( ...
                plant, ...
                postureTimeSeries.q(j), ...
                postureTimeSeries.b(j), ...
                secondaryTask_floor, ...
                secondaryTask_ceiling ...
            );
        else
            stickFigure.update(plant); 
        end
        drawnow;
    end
    close(progressBar);
end
function saveData(hObject, eventdata) %#ok<INUSD>
%     path = '/Users/reimajbi/Neuro/Posture/simulation/data';

    s = 'save plant.mat plant';
    eval(s);
    s = 'save parameters.mat parameters';
    eval(s);
    s = 'save postureTimeSeries.mat postureTimeSeries';
    eval(s);

end
function quitProgram(hObject, eventdata) %#ok<INUSD>
    close all;
end

end % main function































