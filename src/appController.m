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
                self.loopTimer = timer('Period', 0.05, 'BusyMode', 'drop', 'ExecutionMode', ...
                    'fixedDelay');
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
            
            if self.configuration(1) <= 0 && self.arduinoControl.zdir < 0
                self.configuration = self.configuration + [0, -self.arduinoControl.joyX, self.arduinoControl.joyY];
            else
                self.configuration = self.configuration + [self.arduinoControl.zdir, -self.arduinoControl.joyX, self.arduinoControl.joyY];
            end
            
            self.app.Advancement.Text = num2str(self.configuration(3));
            m = 2 * pi;
            self.app.Rotation.Text = num2str(mod(self.configuration(2), m));
            self.app.TendonDisplacement.Text = num2str(self.configuration(1));
            
            % Debugging -- print configuration change values
            % fprintf("| X: %d | Y: %d | Z: %d | C: %d | \n", self.arduinoControl.joyX, self.arduinoControl.joyY, self.arduinoControl.buttonZ, self.arduinoControl.buttonC);
            % fprintf("X: %d | Y: %d | SEL: %d \n", self.arduinoControl.joyX, self.arduinoControl.joyY, self.arduinoControl.joySel);
            
            %% Draw
            try
                %
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
                exception = ME;
            end
            
            if self.error
                self.app.printError(exception);
            else
                self.app.printError([]);
            end
            %hold(self.app.PlotAxes, 'off');
            drawnow limitrate;
        end
        
        function delete(self, obj, event)
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
            %hold(self.app.PlotAxes, 'on');
            
            if self.app.collisionFlag
                if ~isempty(self.app.meMesh)
                    self.detectCollision() % Perform collision detection
                else
                    self.app.CollisionStateLabel.Text = 'No';
                end
            end
            
            if self.app.ToggleHeadVisionCheckBox.Value
                self.drawHead(); % Draw the head projection graph
            end
        end
        
        function detectCollision(self)
            X = rmmissing(X);
            Y = rmmissing(Y);
            Z = rmmissing(Z);
            totalX = X(:);
            totalY = Y(:);
            totalZ = Z(:);
            
            total = [totalX totalY totalZ];
            lol.vertices = total;
            
            % disp("HERERERERE");
            
            % disp("HERERERERE2");
            self.app.meMesh.vertices = self.app.meMesh.Vertices;
            [collisionResult, DistanceNumber, cpobj1, cpobj2]  = CollisionDetection(self.app.meMesh, lol);
            
            if collisionResult
                self.app.CollisionStateLabel.Text = 'Yes';
            else
                [X, Y, Z] = sphere;
                radius = 1;
                XX = X * radius + cpobj1(1);
                YY = Y * radius + cpobj1(2);
                ZZ = Z * radius + cpobj1(3);
                surface(self.app.PlotAxes, XX, YY, ZZ);
                
                XX = X * radius + cpobj2(1);
                YY = Y * radius + cpobj2(2);
                ZZ = Z * radius + cpobj2(3);
                surface(self.app.PlotAxes, XX, YY, ZZ);
                
                self.app.MinimumDistanceNumber.Text = num2str(DistanceNumber);
                self.app.CollisionStateLabel.Text = 'No';
                % K = convhull(self.app.meMesh.me.Vertices)
                % trimesh(self.app.PlotAxes, K, self.app.meMesh.me.Vertices(:,1), self.app.meMesh.me.Vertices(:, 2), self.app.meMesh.me.Vertices(:, 3));
            end
            % hold(self.app.PlotAxes, 'on');
        end
        
        function drawHead(self)
            cla(self.app.HeadAxes);
            copyobj(self.app.PlotAxes.Children, self.app.HeadAxes);
            axis(self.app.HeadAxes, 'equal');
            xi = self.app.wrist.pose(1, end - 1);
            yi = self.app.wrist.pose(2, end - 1);
            zi = self.app.wrist.pose(3, end - 1);
            xf = self.app.wrist.pose(1,end);
            yf = self.app.wrist.pose(2,end);
            zf = self.app.wrist.pose(3,end);
            diffx = xf - xi;
            diffy = yf - yi;
            diffz = zf - zi;
            campos(self.app.HeadAxes, [xf, yf, zf])
            camtarget(self.app.HeadAxes, [xf + diffx, yf + diffy, zf + diffz])
            camva(self.app.HeadAxes, self.app.HeadVisionFOVangleSlider.Value);
        end
    end
end

