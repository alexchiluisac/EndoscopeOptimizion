classdef appController < handle
    %APPCONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        wrist;              % Currently loaded wrist object
        configuration;      % Configuration of the wrist
        % Three-element array
        % [displacement, rotation, advancement]
        app;                % The front-end app
        error;              % Error flag
        arduinoControl;   % Arduino Controller object
        
        initialFlag; % Keep track of first loop
    end
    
    methods
        function self = appController(ID, OD, nCutouts, cutouts)
            %APPCONTROLLER Construct an instance of this class
            %   The controller behind the front-end app
            
            % The wrist object of loaded robot
            self.wrist = Wrist(ID, OD, nCutouts, cutouts);
            
            % The front-end app
            self.app = NotchedDesigner();
            camlight(self.app.PlotAxes, 'headlight');
            
            % Robot configuration
            self.configuration = [self.app.robotDisplacement, self.app.robotRotation, self.app.robotAdvancement];
            
            % Arduino controller
            self.arduinoControl = arduinoController();
        end
        
        % The main update function
        function update(self, obj, event)
            %% Check Termination Flag
            if self.app.stopFlag
                self.delete(0,0); % Terminate the program
            end
            
            %% Updating values
            self.error = 0;
            
            %% CONTROL METHODS
            % KEYBOARD CONTROL
            % self.configuration = [self.app.robotDisplacement, self.app.robotRotation, self.app.robotAdvancement]
            
            % JOYSTICK CONTROL
            % self.arduinoControl.updateValues();
            % self.configuration = self.configuration + [self.arduinoControl.joyX, self.arduinoControl.joyY, 0];
            
            % fprintf("X: %d | Y: %d | SEL: %d \n", self.arduinoControl.joyX, self.arduinoControl.joyY, self.arduinoControl.joySel);
            
            self.arduinoControl.updateNunchukValues();
            self.configuration = self.configuration + [self.arduinoControl.zdir, self.arduinoControl.joyX, self.arduinoControl.joyY];
            
            % Debugging -- print configuration change values
            % fprintf("| X: %d | Y: %d | Z: %d | C: %d | \n", self.arduinoControl.joyX, self.arduinoControl.joyY, self.arduinoControl.buttonZ, self.arduinoControl.buttonC);
            
            %% Draw
            %cla(self.app.PlotAxes);
            
            try
                %                 if ~self.initialFlag
                %                     clear(blackLine);
                %                     clear(redBalls);
                %                     clear(wristSurface);
                %                 else
                %                     self.initialFlag = false;
                %                 end
                % disp(self.app.PlotAxes.Children);
                
                %                 delete(findobj(self.app.PlotAxes.Children.graphics, 'Type','scatter'));
                %                 delete(findobj(self.app.PlotAxes.Children.graphics, 'Type','Surface'));
                %                 delete(findobj(self.app.PlotAxes.Children.graphics, 'Type','Plot'));
                %
                axesHandlesToChildObjects = findobj(self.app.PlotAxes.Children, 'Type', 'Surface');
                if ~isempty(axesHandlesToChildObjects)
                    delete(axesHandlesToChildObjects);
                end
                
                axesHandlesToChildObjects = findobj(self.app.PlotAxes.Children, 'Type', 'Scatter');
                if ~isempty(axesHandlesToChildObjects)
                    delete(axesHandlesToChildObjects);
                end
                axesHandlesToChildObjects = findobj(self.app.PlotAxes.Children, 'Type', 'Line');
                if ~isempty(axesHandlesToChildObjects)
                    delete(axesHandlesToChildObjects);
                end
                
                [blackLine, redBalls, wristSurface] = self.updateGUI();
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
            
            self.app.PlotAxes.XGrid = 'on';
            self.app.PlotAxes.YGrid = 'on';
            self.app.PlotAxes.ZGrid = 'on';
            drawnow;
        end
        
        function delete(self, obj, event)
            stop(obj);
            self.arduinoControl.delete();
            delete(self.arduinoControl);
            self.app.delete();
        end
        
        % Update the gui of the app window
        function [blackLine, redBalls, wristSurface] = updateGUI(self)
            % TODO: replicate the draw function
            
            if ~isempty(self.app.transform)
                self.wrist.fwkine(self.configuration, self.app.transform);
            else
                self.wrist.fwkine(self.configuration, eye(4));
            end
            
            
            X = self.wrist.pose(1,:);
            Y = self.wrist.pose(2,:);
            Z = self.wrist.pose(3,:);
            
            % Draw red circles
            redBalls = scatter3(self.app.PlotAxes, X, Y, Z, 50, 'r', 'filled');
            
            % Draw black line
            blackLine = plot3(self.app.PlotAxes, X, Y, Z, 'k', 'LineWidth', 2.0);
            
            
            if ~isempty(self.app.transform)
                robotModel = self.wrist.makePhysicalModel();
            else
                robotModel = self.wrist.makePhysicalModel();
            end
            X = robotModel.surface.X;
            Y = robotModel.surface.Y;
            Z = robotModel.surface.Z;
            C = X.*Y.*Z;
            wristSurface = surf(self.app.PlotAxes, X, Y, Z, C, 'FaceColor', ...
                'g', 'FaceLighting','gouraud', ...
                'AmbientStrength',0.5, 'EdgeColor', 'k', 'LineWidth', 0.3);
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

