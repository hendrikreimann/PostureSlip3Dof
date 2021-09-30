% class for the display of a stick figure

% the stick figure has real joints and hidden joints, called contact- and
% body degrees of freedom

classdef StickFigure
    properties
        mContactDofs;
        mBodyDofs;
        mNumberOfJoints;
        mSceneBound;
        mFigure;
        mAxes;
        mJointPlots;
        mLinkPlots;
        mScreenPlots;
        mTrackingTargetPlot;
        mTrackerPlot;
    end 
    methods
        function obj = StickFigure(plant)
            obj.mContactDofs = plant.mContactDofs;
            obj.mBodyDofs = plant.mBodyDofs;
            obj.mNumberOfJoints = plant.mNumberOfJoints;
            
            % stick figure
            obj.mSceneBound = [ -0.25; 2.25; 0; 2.5 ];
            obj.mFigure = figure('units', 'normalized', 'Position', [0.7, 0.05, 0.3, 0.5], ...
                    'Name', 'scene');
            obj.mAxes = axes('Position', [ 0.1 0.1 0.8 0.8 ]);
            hold on;
            plot([-10, 10], [0, 0], 'color','k','Linewidth', 3, 'Linestyle','-');
            plot([0, 0], [obj.mSceneBound(3), obj.mSceneBound(4)], 'color','k','Linewidth', 1, 'Linestyle',':');
            obj.mJointPlots = zeros(1, obj.mNumberOfJoints+1);
            obj.mLinkPlots = zeros(1, obj.mNumberOfJoints);
            for l = 1 : obj.mNumberOfJoints
                obj.mJointPlots(l) = plot(0, 0, 'color', 'b', 'Linewidth', 3, 'Marker', 'o');
                obj.mLinkPlots(l) = plot([0, 0], [0, 0], 'color','b','Linewidth',2,'Linestyle','-');
            end
            obj.mJointPlots(obj.mNumberOfJoints+1) = plot(0, 0, 'color','b','Linewidth',3,'Marker','o');
            set(gca,'xlim',[obj.mSceneBound(1), obj.mSceneBound(2)],'ylim',[obj.mSceneBound(3), obj.mSceneBound(4)]);
            
            % screen
            obj.mScreenPlots(1) = plot(0, 0, 'color', 'k', 'Linewidth', 1, 'Linestyle', '-');
            obj.mScreenPlots(2) = plot(0, 0, '+', 'color', 'k', 'Linewidth', 2);
            obj.mTrackingTargetPlot = plot(0, 0, 'o', 'color', 'g', 'Linewidth', 2);
            obj.mTrackerPlot = plot(0, 0, 'color', 'r', 'Linewidth', 1, 'Linestyle', ':');

        end
        
        function update(obj, plant, trackingTarget, trackingBeam, floor, ceiling)
            for i = 1 : obj.mBodyDofs
                set(obj.mJointPlots(i), ...
                        'Xdata', plant.jointPositions(2, i), ...
                        'Ydata', plant.jointPositions(3, i));
                set(obj.mLinkPlots(i), ...
                        'Xdata', [ plant.jointPositions(2, i), plant.jointPositions(2, i+1) ], ...
                        'Ydata', [ plant.jointPositions(3, i), plant.jointPositions(3, i+1) ]);
            end
            headPosition = plant.jointPositions(1:3, obj.mBodyDofs + 1);
            set(obj.mJointPlots(end), 'Xdata', headPosition(2), 'Ydata', headPosition(3) );
           
            % plot secondary task stuff if applicable
            if nargin >= 6
                set(obj.mScreenPlots(1), 'xdata', [2, 2], 'ydata', [0, 2]);
                set(obj.mScreenPlots(2), 'xdata', [2, 2], 'ydata', [floor, ceiling]);
                set(obj.mTrackingTargetPlot, 'xdata', 2, 'ydata', trackingTarget);
                set(obj.mTrackerPlot, ...
                      'xdata', [headPosition(2) 2], ...
                      'ydata', [headPosition(3) trackingBeam] ...
                   )
                    

            else
                set(obj.mScreenPlots(1), 'xdata', [], 'ydata', []);
                set(obj.mScreenPlots(2), 'xdata', [], 'ydata', []);
                set(obj.mTrackingTargetPlot, 'xdata', [], 'ydata', []);
                set(obj.mTrackerPlot, 'xdata', [], 'ydata', []);
            end
        end        
    end
end