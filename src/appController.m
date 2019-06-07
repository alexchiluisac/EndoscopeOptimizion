classdef appController < handle
    %APPCONTROLLER Main controller of the Notched Designer App
    %       Handles the control and drawing of the Notched Designer MATLAB
    %       app as well as the ArduinoNunchuk control interface.
    
    properties
        configuration;      % Configuration of the wrist
        % Three-element array
        % [displacement, rotation, advancement]
        app;                % The front-end app
        error;              % Error flag
        arduinoControl;   % Arduino Controller object
        
        initialFlag; % Keep track of first loop
        
        loopTimer % Looping call-back timer
    end
    
    methods
        function self = appController(ID, OD, nCutouts, cutouts)
            %APPCONTROLLER Construct an instance of this class
            %   The controller behind the front-end app
            
            % The front-end app
            self.app = NotchedDesigner(ID, OD, nCutouts, cutouts);
            camlight(self.app.PlotAxes, 'headlight');
            
            % Robot configuration
            self.configuration = [self.app.robotDisplacement, self.app.robotRotation, self.app.robotAdvancement];
            
            % Arduino controller
            self.arduinoControl = arduinoController();
            
            % Start the application
            self.startApp();
        end
        
        function startApp(self)

            self.app.PlotAxes.XGrid = 'on';
            self.app.PlotAxes.YGrid = 'on';
            self.app.PlotAxes.ZGrid = 'on';
            try
                self.loopTimer = timer('Period', 0.1, 'BusyMode', 'drop', 'ExecutionMode', ...
                    'fixedRate');
                self.loopTimer.TimerFcn = @self.update;
                self.loopTimer.ErrorFcn = @self.delete;
                start(self.loopTimer);
            catch ME
                controller.delete();
                delete(controller);
                rethrow(ME)
            end
            
        end
        
        % The main update function
        function update(self, obj, event)
            %% Check Termination Flag
            if self.app.stopFlag
                stop(self.loopTimer);
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
            
            self.arduinoControl.updateNunchukValues();
            self.configuration = self.configuration + [self.arduinoControl.zdir, -self.arduinoControl.joyX, self.arduinoControl.joyY];
            self.app.Advancement.Text = num2str(self.configuration(3));
            self.app.Rotation.Text = num2str(self.configuration(2));
            self.app.TendonDisplacement.Text = num2str(self.configuration(1));
            
            % Debugging -- print configuration change values
            % fprintf("| X: %d | Y: %d | Z: %d | C: %d | \n", self.arduinoControl.joyX, self.arduinoControl.joyY, self.arduinoControl.buttonZ, self.arduinoControl.buttonC);
            % fprintf("X: %d | Y: %d | SEL: %d \n", self.arduinoControl.joyX, self.arduinoControl.joyY, self.arduinoControl.joySel);
            
            %% Draw
            try
                
                % Delete specific graphical objects from the GUI
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
                
                self.updateSimulation(); % Call the main update function
                
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
        
        function delete(self, obj, event)
            profile viewer;
            delete(self.loopTimer);
            stop(obj);
            self.arduinoControl.delete();
            delete(self.arduinoControl);
            self.app.delete();
        end
        
        % Update the gui of the app window
        function updateSimulation(self)
            % TODO: replicate the draw function
            
            if ~isempty(self.app.transform)
                self.app.wrist.fwkine(self.configuration, self.app.transform);
            else
                self.app.wrist.fwkine(self.configuration, eye(4));
            end
            
            
            X = self.app.wrist.pose(1,:);
            Y = self.app.wrist.pose(2,:);
            Z = self.app.wrist.pose(3,:);
            
            % Draw red circles
            % scatter3(self.app.PlotAxes, X, Y, Z, 50, 'r', 'filled');
            
            % Draw black line
            % plot3(self.app.PlotAxes, X, Y, Z, 'k', 'LineWidth', 2.0);
            
            
            if ~isempty(self.app.transform)
                robotModel = self.app.wrist.makePhysicalModel();
            else
                robotModel = self.app.wrist.makePhysicalModel();
            end
            X = robotModel.surface.X;
            Y = robotModel.surface.Y;
            Z = robotModel.surface.Z;
            surface(self.app.PlotAxes, X, Y, Z, 'FaceColor', ...
                '#5cb5db', 'FaceLighting','gouraud', ...
                'AmbientStrength',0.5, 'EdgeColor', '#585d68', 'LineWidth', 0.003);
            hold(self.app.PlotAxes, 'on');
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

