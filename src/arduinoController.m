classdef arduinoController < handle
    %ARDUINOCONTROLLER Controller and interface for arduino
    %   Reads the necessary values from the arduino
    
    properties
        arduino;    % The arduino object
        joyX = 0;       % The joystick x-axis value
        joyY = 0;       % The joystick y-axis value
        joySel = 0;     % The joystick selector value
    end
    
    methods
        function self = arduinoController()
            %ARDUINOCONTROLLER constructor method
            %   Create an arduino controller
            self.arduino = arduino();
        end
        
        function updateValues(self)
            %UPDATEVALUES Update the values of the arduino object
            %   Read the joystick and other values and store them in the
            %   arduino object.
            
            threshold = 0.1; % Threshold value to avoid drift of joystick
            scaleFactor = 0.1; % To reduce the intensity
            
            tempX = readVoltage(self.arduino, 'A1');
            tempY = readVoltage(self.arduino, 'A0');
            self.joySel = readDigitalPin(self.arduino, 'D2');
            
            fprintf("X: %d | Y: %d | SEL: %d \n", tempX, tempY, self.joySel);
            % Put controller between -1 and 1;
            tempX = (tempX - 2.5) ./ 2.5; 
            tempY = (tempY - 2.5) ./ 2.5;
            fprintf("TempX: %d, TempY: %d \n", tempX, tempY);
            if abs(tempX) > threshold
               self.joyX = tempX .* scaleFactor;
            else
               self.joyX = 0;
            end
            
            if abs(tempY) > threshold
               self.joyY = tempY .* scaleFactor; 
            else
               self.joyY = 0;
            end
            
            
        end
    end
end

