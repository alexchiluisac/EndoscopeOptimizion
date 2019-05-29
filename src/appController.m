classdef appController < handle
    %APPCONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        wrist;          % Currently loaded wrist object
        configuration;  % Configuration of the wrist
                         % Three-element array
                         % [displacement, rotation, advancement]
        app;            % The front-end app
        error;          % Error flag
    end
    
    methods
        function self = appController(ID, OD, nCutouts, cutouts)
            %APPCONTROLLER Construct an instance of this class
            %   The controller behind the front-end app
            
            % The wrist object of loaded robot
            self.wrist = Wrist(ID, OD, nCutouts, cutouts);
            
            % The front-end app
            self.app = NotchedDesigner();
            
            % Robot configuration
            self.configuration = [self.app.robotDisplacement, self.app.robotRotation, self.app.robotAdvancement];
        end
        
        % The main update function
        function update(self)
            %% Updating values
            self.error = 0;
            self.configuration = [self.app.robotDisplacement, self.app.robotRotation, self.app.robotAdvancement]
            
            %% Draw
            
            % Clear the current axes
            cla(self.app.PlotAxes);
            
            try
                
                self.updateGUI();
            catch ME
                
                self.error = 1;
                
                msgID = 'CNTRLR:GUIError';
                msg = 'Unable to update GUI';
                exception = MException(msgID, msg);
                
                % throw(exception);
            end
            
            if self.error
                self.printDiag(msgID ,msg);
            else
                self.printDiag('', '');
            end
        end
        
        
        % Update the gui of the app window
        function updateGUI(self)
            % TODO: replicate the draw function
            [P, T] = self.wrist.fwkine(self.configuration, eye(4));

            X = P(1,:);
            Y = P(2,:);
            Z = P(3,:);
            
            % Draw red circles
            scatter3(self.app.PlotAxes, X, Y, Z, 100, 'r', 'filled');

            % Draw black line
            plot3(self.app.PlotAxes, X, Y, Z, 'k', 'LineWidth', 2.5);

            robotModel = self.wrist.makePhysicalModel(self.configuration, eye(4));

            X = robotModel.surface.X;
            Y = robotModel.surface.Y;
            Z = robotModel.surface.Z;
            
            surf(self.app.PlotAxes, X, Y, Z, 'FaceColor','g');
            % app.PlotAxes.View = [-135 35];
        end
        
        % Print something in the dialog box of the app
        function printDiag(self, errorid, errortext)
            
            if strlength(errortext) > 1
                self.app.ErrorPanel.Visible = 'on';
                self.app.ErrorPanel.Title = 'Error:';
                self.app.ErrorLabel.Text = strcat(errorid, errortext);
            else
                self.app.ErrorPanel.Visible = 'off';
            end
        end
    end
end

