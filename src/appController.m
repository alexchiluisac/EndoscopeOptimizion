classdef appController < handle
    %APPCONTROLLER Main controller of the Notched Designer App
    %       Handles the control and drawing of the Notched Designer MATLAB
    %       app as well as the ArduinoNunchuk control interface.
    
    properties
        configuration;      % Configuration of the wrist
            % Three-element array
            % [displacement, rotation, advancement]
        app;                % Front-end Application Designer App
        error;              % Error flag to display error in app
        arduinoControl;   % Arduino Controller object
            % Controls Arduino and Hardware Control Interface, updates and
            % reads the Nunchuk values from the arduino.
        initialFlag; % Track whether this is the first loop or not
        loopTimer % Looping call-back timer
            % Controls the looping of the entire project
    end
    
    methods
        function self = appController(ID, OD, nCutouts, cutouts)
            %APPCONTROLLER Construct an instance of the appController class
            %   The controller behind the front-end app
            
            % Initialize the front-end app
            self.app = NotchedDesigner(ID, OD, nCutouts, cutouts);
            
            % Configure the robot configuration
            self.configuration = [self.app.robotDisplacement, self.app.robotRotation, self.app.robotAdvancement];
            
            % Initialize the arduino controller
            self.arduinoControl = arduinoController();
            
            % Start the application
            self.startApp();
        end
        
        % Application start-up function
        function startApp(self)
            
            % Error handling statment
            try
                
                % Initialize the looping timer
                self.loopTimer = timer('Period', 0.05, 'BusyMode', 'drop', 'ExecutionMode', ...
                    'fixedDelay');
                
                % Configure the looping function 
                self.loopTimer.TimerFcn = @self.update;
                
                % Set-up and error function to handle termination of the
                % program
                self.loopTimer.ErrorFcn = @self.delete;
                
                % Start the looping timer
                start(self.loopTimer);
            catch ME
                % Error handling
                % Remove all children objects
                controller.delete(); 
                delete(controller);
                rethrow(ME)
            end
            
        end
        
        % UPDATE
        % The main looping function of the entire program
        function update(self, obj, event)
            %% Check Termination Flag
            if self.app.stopFlag
                stop(self.loopTimer);
                self.delete(0,0); % Terminate the program
            end
            
            %% Updating values
            self.error = 0;
            
            %% CONTROL METHODS
            
            % Legacy -- KEYBOARD CONTROL
            % self.configuration = [self.app.robotDisplacement, self.app.robotRotation, self.app.robotAdvancement]
            
            % Legacy -- JOYSTICK CONTROL
            % self.arduinoControl.updateValues();
            % self.configuration = self.configuration + [self.arduinoControl.joyX, self.arduinoControl.joyY, 0];
            
            % Ask arduino to retrieve new Nunchuk values
            self.arduinoControl.updateNunchukValues();
            
            % Update the robot configuration based on control values
            if self.configuration(1) <= 0 && self.arduinoControl.zdir < 0
                self.configuration = self.configuration + [0, -self.arduinoControl.joyX, self.arduinoControl.joyY];
            else
                self.configuration = self.configuration + [self.arduinoControl.zdir, -self.arduinoControl.joyX, self.arduinoControl.joyY];
            end
            
            % Display robot configuration on the App screen
            self.app.Advancement.Text = num2str(self.configuration(3));
            m = 2 * pi;
            self.app.Rotation.Text = num2str(mod(self.configuration(2), m));
            self.app.TendonDisplacement.Text = num2str(self.configuration(1));
            
            % Debugging -- print configuration change values
            % fprintf("| X: %d | Y: %d | Z: %d | C: %d | \n", self.arduinoControl.joyX, self.arduinoControl.joyY, self.arduinoControl.buttonZ, self.arduinoControl.buttonC);
            % fprintf("X: %d | Y: %d | SEL: %d \n", self.arduinoControl.joyX, self.arduinoControl.joyY, self.arduinoControl.joySel);
            
            %% Draw
            
            % Error handling for the graphics and kinematics
            try 
                
                % Delete specific graphical objects from the GUI
                
                % Delete the surface plot of the wrist
                axesHandlesToChildObjects = findobj(self.app.PlotAxes.Children, 'Type', 'Surface');
                if ~isempty(axesHandlesToChildObjects)
                    delete(axesHandlesToChildObjects);
                end
                
                
                % Legacy code -- removing the scatter plot and robot
                % backbone
%                 axesHandlesToChildObjects = findobj(self.app.PlotAxes.Children, 'Type', 'Scatter');
%                 if ~isempty(axesHandlesToChildObjects)
%                     delete(axesHandlesToChildObjects);
%                 end
%                 axesHandlesToChildObjects = findobj(self.app.PlotAxes.Children, 'Type', 'Line');
%                 if ~isempty(axesHandlesToChildObjects)
%                     delete(axesHandlesToChildObjects);
%                 end

                % Update the simulation: Grapics + kinematics
                self.updateSimulation(); 
                
            catch ME
                % Catch any error and display on the app to let user know
                % what is going on
                self.error = 1;
                exception = ME;
            end
            
            % Print the error if there is one
            if self.error
                self.app.printError(exception);
            else
                self.app.printError([]);
            end
            
            % Force the draw, limited to 20 FPS
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
                if ~isempty(self.app.osMesh)
                    self.detectCollision('triangular');
                    % self.detectCollision('GJK'); % Perform
                else
                    self.app.CollisionStateLabel.Text = 'No';
                end
            else
                self.app.CollisionStateLabel.Text = 'No';
            end
            
            if self.app.ToggleHeadVisionCheckBox.Value
                self.drawHead(); % Draw the head projection graph
            end
        end
        
        function detectCollision(self, type)
            if strcmp(type, 'GJK')
                robotModel = self.app.wrist.robotModel;
                X = robotModel.surface.X;
                Y = robotModel.surface.Y;
                Z = robotModel.surface.Z;
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
                [collisionResult, DistanceNumber, cpobj1, cpobj2]  = CollisionDetection(self.app.osMesh, lol);
                
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
            
            if strcmp(type, 'triangular')
                
                robotModel = self.app.wrist.robotModel;
                X = robotModel.surface.X;
                Y = robotModel.surface.Y;
                Z = robotModel.surface.Z;
                X = rmmissing(X);
                Y = rmmissing(Y);
                Z = rmmissing(Z);
                totalX = X(:);
                totalY = Y(:);
                totalZ = Z(:);
                total = [totalX totalY totalZ];
                collision = intriangulation(self.app.osMesh.Vertices, self.app.osMesh.Faces, total);
                collision = sum(collision)
                if collision > 1
                    disp('Collision detected.');
                    
                end
                if collision
                    self.app.CollisionStateLabel.Text = 'Yes';
                    disp("BOOM");
                else
                    self.app.CollisionStateLabel.Text = 'No';
                end
            end
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

