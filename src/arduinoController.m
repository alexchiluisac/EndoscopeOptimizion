classdef arduinoController < handle
    %ARDUINOCONTROLLER Controller and interface for arduino
    %   Reads the necessary values from the arduino
    
    properties (Access = public)
        arduino;    % The arduino object
        
        keyBoard = 0; % Keyboard is only used
        
        % Nunchuk and joystick
        joyX = 0;       % The joystick x-axis value
        joyY = 0;       % The joystick y-axis value
        
        % Joystick
        joySel = 0;     % The joystick selector value
        
        % Nunchuk values
        buttonZ = 0;
        buttonC = 0;
        zdir = 0;
        accX = 0;
        accY = 0;
        accZ = 0;
        
        nunchukAdd;
    end
    
    methods (Access = public)
        function delete(self)
            clear self.arduino;
        end
    end
    
    methods (Access = public)
        function self = arduinoController(comPortString)
            %ARDUINOCONTROLLER constructor method
            %   Create an arduino controller
            try
                % Attempt to load arduino without building, if not possible
                % do a rebuild of the code on the board
                self.arduino = arduino(comPortString, 'Uno', 'Libraries', 'Nunchuk/Nunchuk', 'ForceBuildOn', false, 'Trace', true);
            catch
                disp("Rebuilding Arduino Code");
                self.arduino = arduino(comPortString, 'Uno', 'Libraries', 'Nunchuk/Nunchuk', 'ForceBuildOn', true, 'Trace', true);
            end
            
            self.nunchukAdd = addon(self.arduino, 'Nunchuk/Nunchuk');
            init(self.nunchukAdd);
        end
        
        function updateNunchukValues(self)
            threshold = 0.25; % Threshold value to avoid drift of joystick
            rotScaleFactor = 0.25; % To reduce the intensity of rotation
            advScaleFactor = 0.25; % To reduce the intensity of advancement
            tendonScaleFactor = 0.05; % Reduce intensity of tendon pull
            
            results2 = update(self.nunchukAdd);
            tempX = results2(1); % Store temporary to do more calculations
            tempY = results2(2); % Store temporary to do more calculations
            self.accX = results2(3);
            self.accY = results2(4);
            self.accZ = results2(5);
            self.buttonZ = results2(6);
            self.buttonC = results2(7);
            
            % possible values
            % Joystick
            % X: 20 - 231, rest 123 - 125
            % Y: 25 - 230, rest 127 - 132
            % Accelerometer
            % X: 0 - 1024
            % Y: 0 - 1024
            % Z: 0 - 1024
            % Buttons
            % Z: 0 - 1
            % C: 0 - 1
            
            %% Joystick Values
            
            % Filter out the rest joystick values
            if tempX <= 125 && tempX >= 123
                tempX = 0;
            else
                tempX = (tempX - 125) ./ 107;
            end
            
            % Filter out the rest joystick values
            if tempY <= 132 && tempY >= 127
                tempY = 0;
            else
                tempY = (tempY - 127) ./ 108;
            end
            
            % Joystick Function
            % a(t) = ((50)^t - 1) * 0.025
            
            % Value must be above threshold to move
            if abs(tempX) > threshold
                if tempX > 0
                    self.joyX = (((50 .^ (tempX)) - 1) * 0.025) .* rotScaleFactor;
                else
                    self.joyX = -(((50 .^ (-tempX)) - 1) * 0.025) .* rotScaleFactor;
                end
            else
                self.joyX = 0;
            end
            
            % Value must be above threshold to move
            if abs(tempY) > threshold
                
                if tempY > 0
                    self.joyY = (((50 .^ (tempY)) - 1) * 0.025) .* advScaleFactor;
                else
                    self.joyY = -(((50 .^ (- tempY)) - 1) * 0.025) .* advScaleFactor;
                end
                
            else
                self.joyY = 0;
            end
            
            %% Tendon Values
            if self.buttonC
                % Release tendon
                self.zdir = -1;
            else
                if self.buttonZ
                    % Tension tendon
                    self.zdir = 1;
                else
                    self.zdir = 0;
                end
            end
            
            self.zdir = self.zdir * tendonScaleFactor;
        end
    end
end

