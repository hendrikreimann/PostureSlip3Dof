% class for the display of a stick figure

% the stick figure has real joints and hidden joints, called contact- and
% body degrees of freedom

classdef PostureTimeSeriesFigure
    properties
        headHorizontalPositionPlot;
        headHorizontalVelocityPlot;
        headHorizontalAccelerationPlot;
        uHeadPositionPlot;
        vHeadPositionPlot;
        wHeadPositionPlot;
        trackingTargetPlot;
        trackingBeamPlot;
        copPlot;
        comHorizontalPlot;
        f_jointPositionPlots;
        f_jointVelocityPlots;
        f_comPlots;
        f_torquePlots;
        f_headPlots;
        f_orientationPlots;
        f_totalPlots;
        thetaPlots;
        humanAnglePlots;
        uJointPlots;
        vJointPlots;
        lambdaPlots;
        thetaDotPlots;
        thetaTwoDotPlots;
        muscleTorquePlots;
        stiffnessTorquePlots;
        dampingTorquePlots;
        gravitationalTorquePlots;
        humanAngleData;
        humanTimeData;
        
        mContactDofs;
        mBodyDofs;
        mNumberOfJoints;
        mFigure;
        
        mIsDrawingJoint;
        mIsDrawingLambda = 0;
        mIsDrawingSensor = 0;
        isDrawingHuman = 0;
        isDrawingFTotal = 0;
        isDrawingFJointPosition = 0;
        isDrawingFJointVelocity = 0;
        isDrawingFCom = 1;
        isDrawingFTorque = 0;
        isDrawingFHead = 0;
        isDrawingFOrientation = 0;
        
        mPlotVisibilityCheckBoxes
    end % properties
    methods
        function obj = PostureTimeSeriesFigure(postureTimeSeries)
            figureWidth = 1200;
            figureHeight = 900;
            obj.mContactDofs = postureTimeSeries.mContactDofs;
            obj.mBodyDofs = postureTimeSeries.mBodyDofs;
            obj.mNumberOfJoints = postureTimeSeries.N;
            obj.mFigure = figure('Position', [ 10, 10, figureWidth, figureHeight ], 'Name', 'time series');
            obj.mIsDrawingJoint = ones(1, obj.mBodyDofs);

            % horizontal positions
            subplot(3, 2, 1);
            hold on;
            dots = plot([0, postureTimeSeries.time(end)], [0, 0], 'k:');
            obj.headHorizontalAccelerationPlot = plot(0, 0, 'b-.', 'Linewidth', 2, 'DisplayName', 'acc');
            obj.headHorizontalVelocityPlot = plot(0, 0, 'g--', 'Linewidth', 2, 'DisplayName', 'vel');
            obj.headHorizontalPositionPlot = plot(0, 0, 'r-', 'Linewidth', 2, 'DisplayName', 'pos');
            obj.wHeadPositionPlot = plot(0, 0, 'm-.', 'Linewidth', 1);
            obj.vHeadPositionPlot = plot(0, 0, 'm--', 'Linewidth', 1);
            obj.uHeadPositionPlot = plot(0, 0, 'm-', 'Linewidth', 1, 'DisplayName', 'head sensed');
%             obj.copPlot = plot(0, 0, 'g-', 'Linewidth', 2, 'DisplayName', 'cop');
%             obj.comHorizontalPlot = plot(0, 0, 'b-', 'Linewidth', 2, 'DisplayName', 'com');
%             obj.trackingTargetPlot = plot(0, 0, 'g-.', 'Linewidth', 2, 'DisplayName', 'tracking target');
%             obj.trackingBeamPlot = plot(0, 0, 'r:', 'Linewidth', 2, 'DisplayName', 'tracking beam');


            
            set(get(get(dots, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
%             set(get(get(obj.headHorizontalAccelerationPlot, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
%             set(get(get(obj.headHorizontalVelocityPlot, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
            set(get(get(obj.wHeadPositionPlot, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
            set(get(get(obj.vHeadPositionPlot, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
            legend('toggle');
            title('horizontal positions');

            % forcelets
            subplot(3, 2, 2);
            hold on;
            dots = plot([0, postureTimeSeries.time(end)], [0, 0], 'k:');
            for l = 1 : obj.mBodyDofs
                obj.f_jointPositionPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linewidth', 1, 'Linestyle', '-', 'DisplayName', 'joint pos'); %#ok<*AGROW>
                obj.f_jointVelocityPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linewidth', 1, 'Linestyle', '--', 'DisplayName', 'joint vel'); %#ok<*AGROW>
                obj.f_comPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linewidth', 1, 'Linestyle', '-', 'DisplayName', 'com'); %#ok<*AGROW>
                obj.f_torquePlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linewidth', 1, 'Linestyle', ':', 'DisplayName', 'torque'); %#ok<*AGROW>
                obj.f_headPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linewidth', 1, 'Linestyle', '--', 'DisplayName', 'head'); %#ok<*AGROW>
                obj.f_orientationPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linewidth', 1, 'Linestyle', '-.', 'DisplayName', 'ori'); %#ok<*AGROW>
                obj.f_totalPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linewidth', 2, 'Linestyle', '-', 'DisplayName', 'total');
                set(get(get(obj.f_jointPositionPlots(l), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
                set(get(get(obj.f_jointVelocityPlots(l), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
                set(get(get(obj.f_comPlots(l), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
                set(get(get(obj.f_torquePlots(l), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
                set(get(get(obj.f_headPlots(l), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
                set(get(get(obj.f_orientationPlots(l), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
                set(get(get(obj.f_totalPlots(l), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
            end
%             set(get(get(obj.f_jointPositionPlots(1), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
%             set(get(get(obj.f_jointVelocityPlots(1), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
            set(get(get(obj.f_comPlots(1), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
%             set(get(get(obj.f_torquePlots(1), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
            set(get(get(obj.f_headPlots(1), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
            set(get(get(obj.f_orientationPlots(1), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
            set(get(get(obj.f_totalPlots(1), 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'on');
            set(get(get(dots, 'Annotation'), 'LegendInformation'), 'IconDisplayStyle', 'off');
            legend('toggle');
            title('forcelets');

            % angles
            fullTimeEpisodeStart = 10;
            fullTimeEpisodeLength = postureTimeSeries.T;
%             initialJointAngles = [-.1 .1 -.1];
            initialJointAngles = postureTimeSeries.theta(1:obj.mBodyDofs, 1);
            Fs = 120;
            jointAngles = [];
            load jointAngles_3DoF.mat
            realData_eyesOpen = jointAngles{2, 2, 1};
            episodeStart_human = fullTimeEpisodeStart * Fs + 1;
            episodeEnd_human = min(episodeStart_human + fullTimeEpisodeLength * Fs - 1, size(realData_eyesOpen, 2));
            angles_human = realData_eyesOpen(:, episodeStart_human : episodeEnd_human);
            obj.humanTimeData = 1/Fs : 1/Fs : (episodeEnd_human - episodeStart_human + 1)/Fs;

            subplot(3, 2, 3);
            hold on;
            obj.thetaPlots = zeros(obj.mBodyDofs, 1);
            obj.uJointPlots = zeros(obj.mBodyDofs, 1);
            obj.lambdaPlots = zeros(obj.mBodyDofs, 1);
            obj.humanAngleData = zeros(obj.mBodyDofs, length(obj.humanTimeData));
            for l = 1 : 3
                obj.humanAngleData(l, :) = angles_human(l, :) - mean(angles_human(l, :)) + initialJointAngles(l);
            end
            for l = 4 : obj.mBodyDofs
                obj.humanAngleData(l, :) = zeros(size(angles_human(1, :)));
            end
            for l = 1 : obj.mBodyDofs
                obj.thetaPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linewidth', 2, 'DisplayName', int2str(l));
                obj.humanAnglePlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0] * .5, 'linewidth', 2, 'DisplayName', 'human');
            end
            for l = 1 : obj.mBodyDofs
                obj.uJointPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linestyle', ':', 'Linewidth', 1);
                obj.lambdaPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linestyle', '-.', 'Linewidth', 1);
            end
            plot([0, postureTimeSeries.time(end)], [0, 0], 'k:');
            title('joint angles');

            subplot(3, 2, 4);
            hold on;
            obj.thetaDotPlots = zeros(obj.mBodyDofs, 1);
            obj.vJointPlots = zeros(obj.mBodyDofs, 1);
            for l = 1 : obj.mBodyDofs
                obj.thetaDotPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linewidth', 2);
                obj.vJointPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linestyle', '-.', 'Linewidth', 1);
            end
            plot([0, postureTimeSeries.time(end)], [0, 0], 'k:');
            title('joint velocities');

            subplot(3, 2, 5);
            hold on;
            obj.thetaTwoDotPlots = zeros(obj.mBodyDofs, 1);
            for l = 1 : obj.mBodyDofs
                obj.thetaTwoDotPlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linewidth', 2);
            end
            plot([0, postureTimeSeries.time(end)], [0, 0], 'k:');
            title('joint accelerations');

            subplot(3, 2, 6);
            hold on;
            obj.muscleTorquePlots = zeros(obj.mBodyDofs, 1);
            obj.stiffnessTorquePlots = zeros(obj.mBodyDofs, 1);
            obj.dampingTorquePlots = zeros(obj.mBodyDofs, 1);
            obj.gravitationalTorquePlots = zeros(obj.mBodyDofs, 1);
            for l = 1 : obj.mBodyDofs
                obj.muscleTorquePlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linestyle', '-', 'Linewidth', 2);
                obj.stiffnessTorquePlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linestyle', '--', 'Linewidth', 2);
                obj.dampingTorquePlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linestyle', ':', 'Linewidth', 2);
                obj.gravitationalTorquePlots(l) = plot(0, 0, 'color', [1, (l-1)/obj.mBodyDofs, 0], 'Linestyle', '--', 'Linewidth', 1);
            end
            legend('muscle', 'stiffness', 'damping', 'gravitation');
            plot([0, postureTimeSeries.time(end)], [0, 0], 'k:');
            title('joint torques');
            
        end % constructor  
        function update(obj, postureTimeSeries, index)
            time = postureTimeSeries.time(2:index);
            for i = 1 : obj.mNumberOfJoints - obj.mContactDofs
                % time series
                if obj.mIsDrawingJoint(i)
                    set(obj.thetaPlots(i),   'xdata', time, 'ydata', postureTimeSeries.theta(i+obj.mContactDofs, 2 : index));
                    set(obj.thetaDotPlots(i),   'xdata', time, 'ydata', postureTimeSeries.thetaDot(i+obj.mContactDofs, 2 : index));
                    set(obj.thetaTwoDotPlots(i),   'xdata', time, 'ydata', postureTimeSeries.thetaTwoDot(i+obj.mContactDofs, 2 : index));
                    if obj.isDrawingHuman
                        set(obj.humanAnglePlots(i), 'xdata', obj.humanTimeData,  'ydata', obj.humanAngleData(i, :));
                    else
                        set(obj.humanAnglePlots(i), 'xdata', 0,  'ydata', 0);
                    end
                    if obj.mIsDrawingLambda
                        set(obj.lambdaPlots(i), 'xdata', time, 'ydata', postureTimeSeries.lambda(i, 2 : index));
                    else
                        set(obj.lambdaPlots(i), 'xdata', 0, 'ydata', 0);
                    end
                    if obj.mIsDrawingSensor
                        set(obj.uJointPlots(i),   'xdata', time, 'ydata', postureTimeSeries.uJoint_poly(i, 2 : index));
                        set(obj.vJointPlots(i),   'xdata', time, 'ydata', postureTimeSeries.vJoint_poly(i, 2 : index));
                    else
                        set(obj.uJointPlots(i),   'xdata', 0, 'ydata', 0);
                        set(obj.vJointPlots(i),   'xdata', 0, 'ydata', 0);
                    end

                    set(obj.muscleTorquePlots(i),   'xdata', time, 'ydata', postureTimeSeries.muscleTorque(i, 2 : index));
                    set(obj.stiffnessTorquePlots(i),   'xdata', time, 'ydata', postureTimeSeries.stiffnessTorque(i, 2 : index));
                    set(obj.dampingTorquePlots(i),   'xdata', time, 'ydata', postureTimeSeries.dampingTorque(i, 2 : index));
                    set(obj.gravitationalTorquePlots(i),   'xdata', time, 'ydata', postureTimeSeries.gravitationalTorque(obj.mContactDofs+i, 2 : index));
                    if obj.isDrawingFJointPosition
                        set(obj.f_jointPositionPlots(i), 'xdata', time, 'ydata', postureTimeSeries.f_jointPosition(i, 2 : index));
                    else
                        set(obj.f_jointPositionPlots(i),   'xdata', 0, 'ydata', 0);
                    end
                    if obj.isDrawingFJointVelocity
                        set(obj.f_jointVelocityPlots(i), 'xdata', time, 'ydata', postureTimeSeries.f_jointVelocity(i, 2 : index));
                    else
                        set(obj.f_jointVelocityPlots(i),   'xdata', 0, 'ydata', 0);
                    end
                    if obj.isDrawingFCom
                        set(obj.f_comPlots(i), 'xdata', time, 'ydata', postureTimeSeries.f_com(i, 2 : index));
                    else
                        set(obj.f_comPlots(i),   'xdata', 0, 'ydata', 0);
                    end
                    if obj.isDrawingFTorque
                        set(obj.f_torquePlots(i), 'xdata', time, 'ydata', postureTimeSeries.f_torque(i, 2 : index));
                    else
                        set(obj.f_torquePlots(i),   'xdata', 0, 'ydata', 0);
                    end
                    if obj.isDrawingFHead
                        set(obj.f_headPlots(i), 'xdata', time, 'ydata', postureTimeSeries.f_head(i, 2 : index));
                    else
                        set(obj.f_headPlots(i),   'xdata', 0, 'ydata', 0);
                    end
                    if obj.isDrawingFOrientation
                        set(obj.f_orientationPlots(i), 'xdata', time, 'ydata', postureTimeSeries.f_orientation(i, 2 : index));
                    else
                        set(obj.f_orientationPlots(i),   'xdata', 0, 'ydata', 0);
                    end
                    if obj.isDrawingFTotal
                        set(obj.f_totalPlots(i),   'xdata', time, 'ydata', postureTimeSeries.lambdaDot(i, 2 : index));
                    else
                        set(obj.f_totalPlots(i),   'xdata', 0, 'ydata', 0);
                    end
                else
                    set(obj.thetaPlots(i), 'Xdata', 0, 'Ydata', 0);
                    set(obj.humanAnglePlots(i), 'Xdata', 0, 'ydata', 0);
                    set(obj.lambdaPlots(i), 'Xdata', 0, 'Ydata', 0);
                    set(obj.thetaDotPlots(i), 'Xdata', 0, 'Ydata', 0);
                    set(obj.thetaTwoDotPlots(i), 'Xdata', 0, 'Ydata', 0);
                    set(obj.muscleTorquePlots(i), 'Xdata', 0, 'Ydata', 0);
                    set(obj.stiffnessTorquePlots(i), 'Xdata', 0, 'Ydata', 0);
                    set(obj.dampingTorquePlots(i), 'Xdata', 0, 'Ydata', 0);
                    set(obj.gravitationalTorquePlots(i), 'Xdata', 0, 'Ydata', 0);
                    set(obj.f_jointPositionPlots(i), 'xdata', 0, 'ydata', 0);
                    set(obj.f_jointVelocityPlots(i), 'xdata', 0, 'ydata', 0);
                    set(obj.f_comPlots(i), 'xdata', 0, 'ydata', 0);
                    set(obj.f_torquePlots(i), 'xdata', 0, 'ydata', 0);
                    set(obj.f_headPlots(i), 'xdata', 0, 'ydata', 0);
                    set(obj.f_totalPlots(i), 'Xdata', 0, 'Ydata', 0);
                    set(obj.uJointPlots(i),   'xdata', 0, 'ydata', 0);
                    set(obj.vJointPlots(i),   'xdata', 0, 'ydata', 0);
                end
            end
            set(obj.headHorizontalPositionPlot,   'xdata', time, 'ydata', postureTimeSeries.headPosition_p(2, 2:index));
            set(obj.headHorizontalVelocityPlot,   'xdata', time, 'ydata', postureTimeSeries.headPosition_v(2, 2:index));
%             set(obj.headHorizontalAccelerationPlot,   'xdata', time, 'ydata', postureTimeSeries.headAcceleration(2, 2:index));
            if obj.mIsDrawingSensor
                set(obj.uHeadPositionPlot,   'xdata', time, 'ydata', postureTimeSeries.uHeadPosition(2, 2:index));
                set(obj.vHeadPositionPlot,   'xdata', time, 'ydata', postureTimeSeries.vHeadPosition(2, 2:index));
%                 set(obj.wHeadPositionPlot,   'xdata', time, 'ydata', postureTimeSeries.wHeadPosition(2, 2:index));
            else
                set(obj.uHeadPositionPlot,   'xdata', 0, 'ydata', 0);
                set(obj.vHeadPositionPlot,   'xdata', 0, 'ydata', 0);
%                 set(obj.wHeadPositionPlot,   'xdata', 0, 'ydata', 0);
            end
%             set(obj.copPlot,   'xdata', time, 'ydata', postureTimeSeries.cop(2:index));
%             set(obj.comHorizontalPlot,   'xdata', time, 'ydata', postureTimeSeries.com(2, 2:index));
%                 set(obj.trackingTargetPlot, 'xdata', time, 'ydata', postureTimeSeries.q(2:index));
%                 set(obj.trackingBeamPlot, 'xdata', time, 'ydata', postureTimeSeries.b(2:index));
        end % function update
    end % methods
end