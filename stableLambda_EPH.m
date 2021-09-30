% for a given effective stiffness, calculate the alpha and initial lambda values

function [lambda, steadyStateTorque] = stableLambda_EPH(plant, parameters)


    % calculate desired active torque vector (from muscles activation)

    theta = plant.mJointAngles(plant.mContactDofs+1:end);
    M = plant.mInertiaMatrix(plant.mContactDofs+1:end, plant.mContactDofs+1:end);
    gravitationalTorque = plant.mGravitationalTorques(plant.mContactDofs+1:end);
%     passiveStiffnessTorque = - parameters.K * theta;
    
    factor = 1;
    theta_passive_rad = plant.mJointAngles(plant.mContactDofs+1:end);
    theta_passive_deg = rad2deg(theta_passive_rad);
    theta_passive_deg(3) = - theta_passive_deg(3);
    stiffnessTorque_1 = exp(2.1016 - 0.0843*theta_passive_deg(1) - 0.0176*theta_passive_deg(2)) - exp(- 7.9763 + 0.1949*theta_passive_deg(1) + 0.0008*theta_passive_deg(2)) - 1.792;
    stiffnessTorque_2 = exp(1.800 - 0.0460*theta_passive_deg(1) - 0.0352*theta_passive_deg(2) + 0.0217*theta_passive_deg(3)) - exp(-3.971 - 0.0004*theta_passive_deg(1) + 0.0495*theta_passive_deg(2) - 0.0128*theta_passive_deg(3)) - 4.820 + exp(2.220 - 0.150*theta_passive_deg(2));
    stiffnessTorque_3 = exp(1.4655 - 0.0034*theta_passive_deg(2) - 0.075*theta_passive_deg(3)) - exp(1.3403 - 0.0226*theta_passive_deg(2) + 0.0305*theta_passive_deg(3)) + 8.072;
    passiveStiffnessTorque = factor*[ stiffnessTorque_1; stiffnessTorque_2; stiffnessTorque_3];

    
    desiredActiveTorque = gravitationalTorque - passiveStiffnessTorque;
    desiredActivation = parameters.muscleSetupMatrix^(-1) * desiredActiveTorque;
%     desiredActivation = parameters.muscle_a_ref^(-1) * M^(-1) * desiredActiveTorque;
    
    options = optimset('FinDiffType', 'central', 'MaxFunEvals', 5000, 'TolFun', 1e-12, 'TolX', 1e-12); %#ok<NASGU>
%     options = optimset('MaxFunEvals', 50000, 'TolFun', 1e-2, 'TolX', 1e-2); %#ok<NASGU>
%     lambda = fminunc(@objfun, theta, options);
    [~, lambda] = evalc('fminunc(@objfun, theta, options)');
    
    agonistBase = parameters.muscle_alpha*(theta + parameters.muscle_cc - lambda);
    agonistBase(agonistBase<0) = 0;
    agonistActivation = exp(agonistBase) - 1;
    antagonistBase = - parameters.muscle_alpha*(theta - parameters.muscle_cc - lambda);
    antagonistBase(antagonistBase<0) = 0;
    antagonistActivation = exp(antagonistBase) - 1;
    
    muscleActivation = - agonistActivation + antagonistActivation;
    E = muscleActivation;
%     E = parameters.muscleSetupMatrix^(-1) * M * totalDesiredChange;
    steadyStateTorque = parameters.muscleSetupMatrix * E;
    f = sum((steadyStateTorque + passiveStiffnessTorque - gravitationalTorque).^2);
    
    
function f = objfun(lambda)
    
    
    agonistBase = parameters.muscle_alpha*(theta + parameters.muscle_cc - lambda);
    agonistBase(agonistBase<0) = 0;
    agonistActivation = exp(agonistBase) - 1;
    antagonistBase = - parameters.muscle_alpha*(theta - parameters.muscle_cc - lambda);
    antagonistBase(antagonistBase<0) = 0;
    antagonistActivation = exp(antagonistBase) - 1;
    
    muscleActivation = - agonistActivation + antagonistActivation;

%     E = parameters.muscleSetupMatrix^(-1) * M * totalDesiredChange;
    E = muscleActivation;
    steadyStateTorque = parameters.muscleSetupMatrix * E;
    
%     f = sum((steadyStateTorque + passiveStiffnessTorque - gravitationalTorque).^2);
    f = sum((E - desiredActivation).^2);

end













end





