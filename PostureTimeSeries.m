


classdef PostureTimeSeries
    properties
        time;
        T;
        frequency;
        N;
        mContactDofs;
        mBodyDofs;
        numberOfMuscles;
        L;
        theta;
        thetaDot;
        thetaTwoDot;
        rho;
        rhoDot;
        lambda;
        lambdaDot;
        lambdaTwoDot;
        lambdaTilde;
        lambdaTildeDot;
        
        g_jointVelocity;    % spinal forcelet reducing joint velocities
        g_com;              % horizontal CoM forcelet
        g_torque;           % G-forcelet reducing summed squares of upper joints
        g_visual;           % visual forcelet reducing horizontal head velocity
        f_jointPosition;    % spinal forcelet reducing joint deviation from the reference
        f_jointVelocity;    % spinal forcelet reducing joint velocities
        f_jointAcceleration;    % spinal forcelet reducing joint velocities
        f_com;              % horizontal CoM forcelet
        f_torque;           % G-forcelet reducing summed squares of upper joints
        f_head;           % visual forcelet reducing horizontal head velocity
        f_orientation;           % visual forcelet reducing horizontal head velocity
        f_vol;              % forcelet for voluntary movement
        r_backCoupling;     % reference back-coupling
        J_head;
        J_com;
        inertiaMatrix;
        
        l_CE;
        v_CE;
        a_CE;
        l_CE_hat;
        v_CE_hat;
        a_CE_hat;
        F_CE;
        l_PE;
        F_PE;
        l_SE;
        v_SE;
        F_SE;
        F_isom;
        muscleLambda;
        muscleLambdaDot;
        muscleGammaRelative;
        muscleStimulation;
        muscleActivation;
        muscleTorque;
        muscleTorqueDot;
        muscleTorqueTwoDot;
        appliedTorque;
        limitTorque;
        l_MTC;
        momentArms;
        transformationMatrices;
        
        externalTorque;
        stiffnessTorque;
        dampingTorque;
        gravitationalTorque;
        gravitationalTorqueDot;
        groundReactionForces;
        
        headPosition_p;
        headPosition_v;
        headPosition_a;
        headOrientation_p;
        headOrientation_v;
        headOrientation_a;
        
        q;                  % secondary task target position
        qDot;               % secondary task target velocity
        qTwoDot;            % secondary task target acceleration
        b;                  % secondary task beam position
        bDot;               % secondary task beam velocity
        bTwoDot;            % secondary task beam acceleration
        
        uJoint_mono;        % estimate of current joint angles
        vJoint_mono;        % estimate of current joint velocities
        wJoint_mono;        % estimate of current joint accelerations
        uJoint_poly;        % estimate of current joint angles
        vJoint_poly;        % estimate of current joint velocities
        wJoint_poly;        % estimate of current joint accelerations
        uJoint_central;     % estimate of current joint angles
        vJoint_central;     % estimate of current joint velocities
        wJoint_central;     % estimate of current joint accelerations
        xJoint;             % sensed joint angle
        yJoint;             % sensed joint velocity
        zJoint;             % sensed joint acceleration

        uHeadPosition;              % estimate of current head position
        vHeadPosition;              % estimate of current head velocity
        wHeadPosition;              % estimate of current head acceleration
        xHeadPosition;              % sensed head position
        yHeadPosition;              % sensed head velocity
        zHeadPosition;              % sensed head acceleration

        uHeadOrientation;              % estimate of current head position
        vHeadOrientation;              % estimate of current head velocity
        wHeadOrientation;              % estimate of current head acceleration
        xHeadOrientation;              % sensed head position
        yHeadOrientation;              % sensed head velocity
        zHeadOrientation;              % sensed head acceleration
        
        uTrackingError;     % estimate of current tracking error in position
        vTrackingError;     % estimate of current tracking error in velocity
        wTrackingError;     % estimate of current tracking error in acceleration
        xTrackingError;     % sensed tracking error in position
        yTrackingError;     % sensed tracking error in velocity
        zTrackingError;     % sensed tracking error in acceleration
        
        xCom;               % sensed center of mass position in ap-direction (y-axis)
        yCom;               % sensed center of mass velocity in ap-direction (y-axis)
        zCom;               % sensed center of mass acceleration in ap-direction (y-axis)
        uCom;               % estimate of current center of mass position in ap-direction (y-axis)
        vCom;               % estimate of current center of mass velocity in ap-direction (y-axis)
        wCom;               % estimate of current center of mass acceleration in ap-direction (y-axis)
        xCop;               % sensed center of pressure in ap-direction (y-axis)
        uTorque;            % estimate of joint torque vector
        xTorque;            % sensed joint torque vector

        com;                % center of mass position
        comDot;             % center of mass velocity
        comTwoDot;          % center of mass acceleration
        cop;                % center of pressure position along the y-axis
        copDot;             % rate of change of center of pressure along the y-axis
        copTwoDot;          % acceleration of center of pressure along the y-axis
        
        vtc_pos;            % virtual time to contact with positive stability limit
        vtc_neg;            % virtual time to contact with negative stability limit
    end
    methods
        function obj = PostureTimeSeries(frequency, totalTimeSteps, plant, numberOfMuscles)
            obj.time = (1 : totalTimeSteps) / frequency;
            obj.T = totalTimeSteps/frequency;
            obj.L = length(obj.time);
            obj.frequency = frequency;
            obj.N = plant.mNumberOfJoints;
            obj.mContactDofs = plant.mContactDofs;
            obj.mBodyDofs = plant.mBodyDofs;
            obj.numberOfMuscles = numberOfMuscles;
            obj.theta = zeros(obj.N, obj.L);
            obj.thetaDot = zeros(obj.N, obj.L);
            obj.thetaTwoDot = zeros(obj.N, obj.L);
            obj.rho = zeros(obj.mBodyDofs, obj.L);
            obj.rhoDot = zeros(obj.mBodyDofs, obj.L);
            obj.lambda = zeros(obj.mBodyDofs, obj.L);
            obj.lambdaDot = zeros(obj.mBodyDofs, obj.L);
            obj.lambdaTwoDot = zeros(obj.mBodyDofs, obj.L);
            obj.lambdaTilde = zeros(obj.mBodyDofs, obj.L);
            obj.lambdaTildeDot = zeros(obj.mBodyDofs, obj.L);
            
            obj.g_jointVelocity = zeros(obj.mBodyDofs, obj.L);
            obj.g_com = zeros(1, obj.L);
            obj.g_torque = zeros(1, obj.L);
            obj.g_visual = zeros(1, obj.L);
            obj.f_jointPosition = zeros(obj.mBodyDofs, obj.L);
            obj.f_jointVelocity = zeros(obj.mBodyDofs, obj.L);
            obj.f_jointAcceleration = zeros(obj.mBodyDofs, obj.L);
            obj.f_com = zeros(obj.mBodyDofs, obj.L);
            obj.f_torque = zeros(obj.mBodyDofs, obj.L);
            obj.f_head = zeros(obj.mBodyDofs, obj.L);
            obj.f_vol = zeros(obj.mBodyDofs, obj.L);
            obj.f_orientation = zeros(obj.mBodyDofs, obj.L);
            obj.r_backCoupling = zeros(obj.mBodyDofs, obj.L);
            obj.J_head = zeros(obj.mBodyDofs, obj.L);
            obj.J_com = zeros(obj.mBodyDofs, obj.L);
            obj.inertiaMatrix = zeros(obj.mBodyDofs, obj.mBodyDofs, obj.L);
            
            obj.l_CE = zeros(obj.numberOfMuscles, obj.L);
            obj.v_CE = zeros(obj.numberOfMuscles, obj.L);
            obj.a_CE = zeros(obj.numberOfMuscles, obj.L);
            obj.l_CE_hat = zeros(obj.numberOfMuscles, obj.L);
            obj.v_CE_hat = zeros(obj.numberOfMuscles, obj.L);
            obj.a_CE_hat = zeros(obj.numberOfMuscles, obj.L);
            obj.F_CE = zeros(obj.numberOfMuscles, obj.L);
            obj.l_PE = zeros(obj.numberOfMuscles, obj.L);
            obj.F_PE = zeros(obj.numberOfMuscles, obj.L);
            obj.l_SE = zeros(obj.numberOfMuscles, obj.L);
            obj.v_SE = zeros(obj.numberOfMuscles, obj.L);
            obj.F_SE = zeros(obj.numberOfMuscles, obj.L);
            obj.F_isom = zeros(obj.numberOfMuscles, obj.L);
            obj.muscleLambda = zeros(obj.numberOfMuscles, obj.L);
            obj.muscleLambdaDot = zeros(obj.numberOfMuscles, obj.L);
            obj.muscleGammaRelative = zeros(obj.numberOfMuscles, obj.L);
            obj.muscleStimulation = zeros(obj.numberOfMuscles, obj.L);
            obj.muscleActivation = zeros(obj.numberOfMuscles, obj.L);
            obj.muscleTorque = zeros(obj.mBodyDofs, obj.L);
            obj.muscleTorqueDot = zeros(obj.mBodyDofs, obj.L);
            obj.muscleTorqueTwoDot = zeros(obj.mBodyDofs, obj.L);
            obj.limitTorque = zeros(obj.mBodyDofs, obj.L);
            obj.l_MTC = zeros(obj.numberOfMuscles, obj.L);
            obj.momentArms = zeros(obj.numberOfMuscles, obj.mBodyDofs, obj.L);
            obj.transformationMatrices = zeros(obj.numberOfMuscles, obj.mBodyDofs, obj.L);

            obj.appliedTorque = zeros(obj.mBodyDofs, obj.L);
            obj.stiffnessTorque = zeros(obj.mBodyDofs, obj.L);
            obj.dampingTorque = zeros(obj.mBodyDofs, obj.L);
            obj.externalTorque = zeros(obj.N, obj.L);
            obj.gravitationalTorque = zeros(obj.N, obj.L);
            obj.gravitationalTorqueDot = zeros(obj.N, obj.L);
            obj.groundReactionForces = zeros(obj.mContactDofs, obj.L);
            
            obj.headPosition_p = zeros(3, obj.L);
            obj.headPosition_v = zeros(3, obj.L);
            obj.headPosition_a = zeros(3, obj.L);
            obj.headOrientation_p = zeros(1, obj.L);
            obj.headOrientation_v = zeros(1, obj.L);
            obj.headOrientation_a = zeros(1, obj.L);
            obj.q = zeros(1, obj.L);
            obj.qDot = zeros(1, obj.L);
            obj.qTwoDot = zeros(1, obj.L);
            obj.b = zeros(1, obj.L);
            obj.bDot = zeros(1, obj.L);
            obj.bTwoDot = zeros(1, obj.L);
            
            
            obj.uJoint_mono = zeros(obj.mBodyDofs, obj.L);
            obj.vJoint_mono = zeros(obj.mBodyDofs, obj.L);
            obj.wJoint_mono = zeros(obj.mBodyDofs, obj.L);
            obj.uJoint_poly = zeros(obj.mBodyDofs, obj.L);
            obj.vJoint_poly = zeros(obj.mBodyDofs, obj.L);
            obj.wJoint_poly = zeros(obj.mBodyDofs, obj.L);
            obj.uJoint_central = zeros(obj.mBodyDofs, obj.L);
            obj.vJoint_central = zeros(obj.mBodyDofs, obj.L);
            obj.wJoint_central = zeros(obj.mBodyDofs, obj.L);
            obj.xJoint = zeros(obj.mBodyDofs, obj.L);
            obj.yJoint = zeros(obj.mBodyDofs, obj.L);
            obj.zJoint = zeros(obj.mBodyDofs, obj.L);

            obj.uTorque = zeros(obj.mBodyDofs, obj.L);
            obj.xTorque = zeros(obj.mBodyDofs, obj.L);

            obj.uHeadPosition = zeros(3, obj.L);
            obj.vHeadPosition = zeros(3, obj.L);
            obj.wHeadPosition = zeros(3, obj.L);
            obj.xHeadPosition = zeros(3, obj.L);
            obj.yHeadPosition = zeros(3, obj.L);
            obj.zHeadPosition = zeros(3, obj.L);

            obj.uHeadOrientation = zeros(1, obj.L);
            obj.vHeadOrientation = zeros(1, obj.L);
            obj.wHeadOrientation = zeros(1, obj.L);
            obj.xHeadOrientation = zeros(1, obj.L);
            obj.yHeadOrientation = zeros(1, obj.L);
            obj.zHeadOrientation = zeros(1, obj.L);

            obj.uTrackingError = zeros(1, obj.L);
            obj.vTrackingError = zeros(1, obj.L);
            obj.wTrackingError = zeros(1, obj.L);
            obj.uTrackingError(1) = obj.b(1) - obj.q(1);
            obj.vTrackingError(1) = obj.bDot(1) - obj.qDot(1);
            obj.wTrackingError(1) = obj.bTwoDot(1) - obj.qTwoDot(1);
            obj.xTrackingError = zeros(1, obj.L);
            obj.yTrackingError = zeros(1, obj.L);
            obj.zTrackingError = zeros(1, obj.L);
            obj.xCom = zeros(1, obj.L);
            obj.yCom = zeros(1, obj.L);
            obj.zCom = zeros(1, obj.L);
            obj.uCom = zeros(1, obj.L);
            obj.vCom = zeros(1, obj.L);
            obj.wCom = zeros(1, obj.L);
            obj.xCop = zeros(1, obj.L);
            
            obj.com = zeros(3, obj.L);
            obj.comDot = zeros(3, obj.L);
            obj.comTwoDot = zeros(3, obj.L);
            obj.cop = zeros(1, obj.L);
            obj.copDot = zeros(1, obj.L);
            obj.copTwoDot = zeros(1, obj.L);
            
            obj.vtc_pos = zeros(1, obj.L);
            obj.vtc_neg = zeros(1, obj.L);
        end
    end
end