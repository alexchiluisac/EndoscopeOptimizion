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

            
%             theta = 2*pi*linspace(0,2,100);
%             x = cos(theta);
%             y = sin(theta);
%             z = theta/(2*pi);
%             [T,N,B,k,t] = frenet(x,y,z);
%             line(self.app.PlotAxes, x,y,z)
%             quiver3(self.app.PlotAxes, x, y, z, T(:,1)',T(:,2)',T(:,3)','color','r')
%             quiver3(self.app.PlotAxes, x,y,z, N(:,1)',N(:,2)',N(:,3)','color','g')
%             quiver3(self.app.PlotAxes, x,y,z, B(:,1)',B(:,2)',B(:,3)','color','b')
        end
        
        function draw(self)
            self.app.draw();
        end
        
        function update(self)
            if self.app.startWaitFlag
                self.app.arduinoController.updateNunchukValues();
            end
            self.app.generateCurve();
        end
    end
    
end
