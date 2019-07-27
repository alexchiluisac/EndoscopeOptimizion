classdef synthesisController < handle
    % SYNTHESISCONTROLLER The main controller for the synthesis app
    
    properties
        app; % The MLAPP app
        scatterPlot;
    end
    
    methods
        function self = synthesisController()
            self.app = SynthesisApp();
            xlabel(self.app.PlotAxes, 'X [m]'), ylabel(self.app.PlotAxes, 'Y [m]'), zlabel(self.app.PlotAxes, 'Z [m]');

%             while true
%                self.update()
%                self.draw()
%                pause(0.5); % Update twice per second
%             end
        end
        
        function draw(self)
            self.app.draw();
        end
        
        function update(self)
            self.app.update();
        end
    end
    
end
