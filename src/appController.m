classdef appController < handle
    %APPCONTROLLER Main controller of the Notched Designer App
    %       Handles the control and drawing of the Notched Designer MATLAB
    %       app as well as the ArduinoNunchuk control interface.
    
    properties
        app;                % Front-end Application Designer App
        error;              % Error flag to display error in app
        
        % Separate timers for different loops
        loopTimer           % Looping call-back timer
        drawingTimer        % Looping draw timer
        controlsTimer       % Get controller values
        
        % For debugging
        loopCounter = 0;
        
        % Different graphical objects stored here
        rayCastingPatch;    % Ray casting model
        collisionScatter;   % Collision scatter plot
        robotSurface;       % Robot surface model
    end
    
    methods
        function self = appController(ID, OD, nCutouts, cutouts)
            %APPCONTROLLER Construct an instance of the appController class
            %   The controller behind the front-end app
            
            % Initialize the front-end app (.mlapp)
            self.app = NotchedDesigner(ID, OD, nCutouts, cutouts);
            
            % Configure the robot configuration
            self.app.configuration = [self.app.robotDisplacement, self.app.robotRotation, self.app.robotAdvancement];
            
            % Initialize the arduino controller
            % self.arduinoControl = arduinoController();
            
            self.app.wrist.fwkine(self.app.configuration, eye(4));
            self.app.wrist.makePhysicalModel();
            
            robotModel = self.app.wrist.robotModel;
            
            % Draw the robot surface model
            X = robotModel.surface.X;
            Y = robotModel.surface.Y;
            Z = robotModel.surface.Z;
            
            % Draw the new robot surface
            self.robotSurface = surface(self.app.PlotAxes, X, Y, Z, 'FaceColor', ...
                '#5cb5db', ...
                'AmbientStrength',0.5, 'EdgeColor', '#585d68', 'LineWidth', 0.003);
            comME = MException("APPCONTROLLER:enterCOM", "Enter COM Port number and press Connect Arduino! \n");
            self.app.printError(comME);
            while isempty(self.app.arduinoController)
                % Do nothing
                pause(0.05);
            end
            
            self.app.printError([]);
            % Start the application
            %self.startApp();
            
            % For debugging use loopingAlternative
            % MATLAB does not have proper error statements for timers
            self.loopingAlternative();
            
        end
        
        % An alternative to the timer application
        function loopingAlternative(self)
            
            while ~self.app.stopFlag
                self.updateValues(0, 0);
                self.updateGraphics(0, 0);
                if ~(self.app.arduinoController.keyBoard)
                    self.updateControls(0, 0);
                end
                self.loopCounter = self.loopCounter + 1;
                %disp(self.loopCounter);
                pause(0.05);
            end
            
        end
        
        % Application start-up function
        function startApp(self)
            % Error handling statment
            try
                % Initialize the main looping timer
                self.loopTimer = timer('Period', 0.10, 'BusyMode', 'drop', 'ExecutionMode', ...
                    'fixedRate');
                
                % Configure the looping function
                self.loopTimer.TimerFcn = @self.updateValues;
                
                % Set-up and error function to handle termination of the
                % program
                self.loopTimer.ErrorFcn = @self.delete;
                
                % Initialize the drawing timer
                self.drawingTimer = timer('Period', 0.1, 'BusyMode', 'drop', 'ExecutionMode', ...
                    'fixedRate');
                
                % Configure the looping function
                self.drawingTimer.TimerFcn = @self.updateGraphics;
                
                % Set-up and error function to handle termination of the
                % program
                self.drawingTimer.ErrorFcn = @self.delete;
                
                start(self.loopTimer);
                start(self.drawingTimer);
                
                if ~(self.app.arduinoController.keyBoard)
                    % Initialize the control timer
                    self.controlsTimer = timer('Period', 0.12, 'BusyMode', 'drop', 'ExecutionMode', ...
                        'fixedRate');
                    
                    % Configure the looping function
                    self.controlsTimer.TimerFcn = @self.updateControls;
                    
                    % Set-up and error function to handle termination of the
                    % program
                    self.controlsTimer.ErrorFcn = @self.delete;
                    
                    %                     comME = MException("APPCONTROLLER:keyboardEnabled", "Keyboard enabled!\n");
                    %                     self.app.printError(comME);
                    disp("Keyboard mode enabled");
                end
                % Start the looping timer
                
                start(self.controlsTimer);
            catch ME
                Error handling
                Remove all children objects
                controller.delete();
                delete(controller);
                rethrow(ME)
            end
            
        end
        
        % UPDATECONTROLS
        % Used in timer callback loop to update controls
        function updateControls(self, obj, event)
            
            if self.app.stopFlag
                self.delete(self.loopTimer,0); % Terminate the program
            end
            
            % Ask arduino to retrieve new Nunchuk values
            self.app.arduinoController.updateNunchukValues();
            %             self.arduinoControl.zdir = 0;
            %             self.arduinoControl.joyX = 0;
            %             self.arduinoControl.joyY = 0;
            
            % Update the robot configuration based on control values
            % Only if the robot tendon displacement is greater than 0
            if self.app.configuration(1) <= 0 && self.app.arduinoController.zdir < 0
                self.app.configuration = self.app.configuration + [0, 0, 0]; % Do nothing
            else
                self.app.configuration = self.app.configuration + [self.app.arduinoController.zdir, 0, 0];
            end
            
            % Update robot advancement if value is greater than 0 mm.
            if self.app.configuration(3) <= 0 && self.app.arduinoController.joyY < 0
                self.app.configuration = self.app.configuration + [0, 0, 0]; % Do nothing
            else
                self.app.configuration = self.app.configuration + [0, 0, self.app.arduinoController.joyY];
            end
            
            % Rotation can always be updated
            self.app.configuration = self.app.configuration + [0, -self.app.arduinoController.joyX, 0];
            
            % Debugging -- print configuration change values
            % fprintf("| X: %d | Y: %d | Z: %d | C: %d | \n", self.arduinoControl.joyX, self.arduinoControl.joyY, self.arduinoControl.buttonZ, self.arduinoControl.buttonC);
            % fprintf("X: %d | Y: %d | SEL: %d \n", self.arduinoControl.joyX, self.arduinoControl.joyY, self.arduinoControl.joySel);
            
        end
        
        
        % UPDATEGRAPHICS
        % Timer callback loop function to update graphics
        function updateGraphics(self, obj, event)
            % Right now only the robot surface model is updated in this
            % loop.
            
            if self.app.stopFlag
                self.delete(self.loopTimer,0); % Terminate the program
            end
            
            % Display robot configuration on the App screen
            self.app.Advancement.Text = num2str(self.app.configuration(3));
            m = 2 * pi;
            self.app.Rotation.Text = num2str(mod(self.app.configuration(2), m));
            self.app.TendonDisplacement.Text = num2str(self.app.configuration(1));
            
            robotModel = self.app.wrist.robotModel;
            
            % Draw the robot surface model
            X = robotModel.surface.X;
            Y = robotModel.surface.Y;
            Z = robotModel.surface.Z;
            
            % Delete the previous drawing
            % delete(self.robotSurface);
            
            % Update new robot surface data
            set(self.robotSurface, 'XData', X);
            set(self.robotSurface, 'YData', Y);
            set(self.robotSurface, 'ZData', Z);
            
            hold(self.app.PlotAxes, 'off');
            
        end
        
        % UPDATEVALUES
        % The main looping function of the entire program for data
        function updateValues(self, obj, event)
            %% Check Termination Flag
            if self.app.stopFlag
                self.delete(self.loopTimer,0); % Terminate the program
            end
            
            % disp(self.app.PlotAxes.Children)
            %% Updating values
            self.error = 0;
            
            %% Draw
            
            % Error handling for the graphics and kinematics
            try
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
        end
        
        % DELETE
        % Delete this object and all its children
        function delete(self, obj, event)
            stop(obj); % Stop the timer
            self.app.arduinoController.delete(); % Delete the arduino controller children
            delete(self.app.arduinoController); % Delete the arduino controller
            self.app.delete(); % Delete the app
        end
        
        % UPDATESIMULATION
        % Update the Simulation and Kinematics of the app
        function updateSimulation(self)
            hold(self.app.PlotAxes, 'on'); % Hold the drawing
            
            % Perform transform if necessary
            if ~isempty(self.app.transform)
                % STL is loaded, draw wrist in STL
                self.app.wrist.fwkine(self.app.configuration, self.app.transform);
            else
                % STL is not loaded, draw wrist at origin (0,0,0)
                self.app.wrist.fwkine(self.app.configuration, eye(4));
            end
            
            
            self.app.wrist.makePhysicalModel();
            
            % Check if collision detection is necessary
            if self.app.collisionFlag
                
                % Only perform collision detection if there is an ossicle
                % mesh loaded
                if ~isempty(self.app.osMesh)
                    % Perform triangular collision detection
                    self.detectCollision('triangular');
                    
                    % Perform GJK collision detection -- not working yet
                    % self.detectCollision('GJK');
                else
                    self.app.CollisionStateLabel.Text = 'No Model Loaded';
                end
            else
                % Switch text to off
                self.app.CollisionStateLabel.Text = 'Off';
            end
            
            % Check if the head needs to be drawn
            if self.app.ToggleHeadVisionCheckBox.Value
                self.drawHead(); % Draw the head projection graph
            end
            
            % Only perform ray-trace visibility if there is a model loaded
            if self.app.RayTraceAreaVisibilityCheckBox.Value && ~isempty(self.app.osMesh)
                
                % Perform ray tracing, return hit faces and vertices
                [faces, vertices] = self.checkRayTrace();
                faces = faces';
                
                result = self.app.totalMesh.Faces .* faces;
                filtered = result(all(result,2), :);
                delete(self.rayCastingPatch);
                self.rayCastingPatch = patch(self.app.PlotAxes, 'Faces', filtered, ...
                    'Vertices', self.app.totalMesh.Vertices, 'FaceColor', ...
                    '#fcf92f', 'FaceLighting','gouraud', ...
                    'AmbientStrength',0.5, 'EdgeColor', '#585d68', ...
                    'LineWidth', 0.003, 'FaceAlpha', 0.8);
                
                self.app.RayTraceAreaVisibilityCheckBox.Value = false;
            end
            
            % Draw the red line coming out of the robot head
            if self.app.ToggleVisionLineCheckBox.Value
                self.drawVisionLine();
            end
        end
        
        
        % CHECKRAYTRACE
        % Check the visibility of the models to the wrist
        function [seenFaces, seenVertices] = checkRayTrace(self)
            
            self.app.totalMesh.vertices = self.app.totalMesh.Vertices';
            self.app.totalMesh.faces = self.app.totalMesh.Faces';
            
            % Location behind the head
            xi = self.app.wrist.pose(1, end - 1);
            yi = self.app.wrist.pose(2, end - 1);
            zi = self.app.wrist.pose(3, end - 1);
            
            % Location of the head
            xf = self.app.wrist.pose(1,end);
            yf = self.app.wrist.pose(2,end);
            zf = self.app.wrist.pose(3,end);
            
            % Difference between the two
            diffx = xf - xi;
            diffy = yf - yi;
            diffz = zf - zi;
            
            % Calculate the unit vector originating from the head of the
            % wrist
            magVec = sqrt((diffx)^2 + (diffy)^2 + (diffz)^2);
            unitVec = [diffx/magVec, diffy/magVec, diffz/magVec];
            [seenFaces, seenVertices] = visibilitymap([xf yf zf], unitVec, self.app.totalMesh);
        end
        
        % DETECTCOLLISION
        % type - string - Representing the desired collision option
        % 'GJK' - For GJK collision detection
        % 'triangular' - For intriangulation collision detection
        function detectCollision(self, type)
            if strcmp(type, 'GJK')
                % Perform GJK collision detection
                % TODO: Fix this, make it compatible with concave shapes
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
                    hold(self.app.PlotAxes, 'on');
                    XX = X * radius + cpobj2(1);
                    YY = Y * radius + cpobj2(2);
                    ZZ = Z * radius + cpobj2(3);
                    surface(self.app.PlotAxes, XX, YY, ZZ);
                    hold(self.app.PlotAxes, 'on');
                    self.app.MinimumDistanceNumber.Text = num2str(DistanceNumber);
                    self.app.CollisionStateLabel.Text = 'No';
                    % K = convhull(self.app.meMesh.me.Vertices)
                    % trimesh(self.app.PlotAxes, K, self.app.meMesh.me.Vertices(:,1), self.app.meMesh.me.Vertices(:, 2), self.app.meMesh.me.Vertices(:, 3));
                end
                % hold(self.app.PlotAxes, 'on');
            end
            
            if strcmp(type, 'triangular')
                % Perform intriangulation collision detection
                
                % Load the robot model points
                robotModel = self.app.wrist.robotModel;
                X = robotModel.surface.X;
                Y = robotModel.surface.Y;
                Z = robotModel.surface.Z;
                
                % Remove the NaN values
                X = rmmissing(X);
                Y = rmmissing(Y);
                Z = rmmissing(Z);
                
                % Put the data in a [n x 3] matrix
                totalX = X(:);
                totalY = Y(:);
                totalZ = Z(:);
                total = [totalX totalY totalZ];
                
                % Call the intriangulation collsion detection
                [collision, points] = intriangulation(self.app.osMesh.Vertices, self.app.osMesh.Faces, total);
                collision = sum(collision);
                
                % Draw the collision points on the plot
                if ~isempty(points)
                    delete(self.collisionScatter);
                    hold(self.app.PlotAxes, 'on');
                    self.collisionScatter = scatter3(self.app.PlotAxes, ...
                        points(:, 1), points(:, 2), points(:, 3), ...
                        5, 'filled', 'MarkerFaceColor', '#e22424');
                end
                
                if collision
                    self.app.CollisionStateLabel.Text = 'Yes';
                else
                    self.app.CollisionStateLabel.Text = 'No';
                end
            end
        end
        
        % DRAWHEAD
        % Display the vision of the head of the wrist
        function drawHead(self)
            cla(self.app.HeadAxes); % Clear the current axes
            
            % Copy the graphical components of the current plot to the new
            % head plot
            copyobj(self.app.PlotAxes.Children, self.app.HeadAxes);
            
            axis(self.app.HeadAxes, 'equal');
            
            % Math to choose where the camera should be
            
            % Location behind the head
            xi = self.app.wrist.pose(1, end - 1);
            yi = self.app.wrist.pose(2, end - 1);
            zi = self.app.wrist.pose(3, end - 1);
            
            % Location of the head
            xf = self.app.wrist.pose(1,end);
            yf = self.app.wrist.pose(2,end);
            zf = self.app.wrist.pose(3,end);
            
            % Difference between the two
            diffx = xf - xi;
            diffy = yf - yi;
            diffz = zf - zi;
            
            % Place the camera on the head, and point it in front of the
            % head
            campos(self.app.HeadAxes, [xf + diffx / 100, yf + diffy / 100, zf  + diffz / 100]);
            camtarget(self.app.HeadAxes, [xf + diffx, yf + diffy, zf + diffz]);
            
            % Change the FOV depending on the FOV slider on the UI
            camva(self.app.HeadAxes, self.app.HeadVisionFOVangleSlider.Value);
        end
        
        % DRAWVISIONLINE
        % Draw a line representation of the orientation of the head of the
        % wrist of the robot.
        function drawVisionLine(self)
            length = 10;
            % Location behind the head
            xi = self.app.wrist.pose(1, end - 1);
            yi = self.app.wrist.pose(2, end - 1);
            zi = self.app.wrist.pose(3, end - 1);
            
            % Location of the head
            xf = self.app.wrist.pose(1,end);
            yf = self.app.wrist.pose(2,end);
            zf = self.app.wrist.pose(3,end);
            
            % Difference between the two
            diffx = xf - xi;
            diffy = yf - yi;
            diffz = zf - zi;
            
            % Calculate the unit vector originating from the head
            n = norm([diffx diffy diffz]);
            unit_x = diffx / n;
            unit_y = diffy / n;
            unit_z = diffz / n;
            
            % Calculate the start and end point of the vision line
            X = [xi (xi + length * unit_x)];
            Y = [yi (yi + length * unit_y)];
            Z = [zi (zi + length * unit_z)];
            hold(self.app.PlotAxes, 'on');
            delete(self.app.visionLine);
            self.app.visionLine = plot3(self.app.PlotAxes, X, Y, Z, 'Color', '#e51919', ...
                'LineWidth', 0.3);
        end
    end
end

