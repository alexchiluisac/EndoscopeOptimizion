classdef NunchukAddon < matlabshared.addon.LibraryBase
    % NUNCHUKADDON Custom Add-on for Arduino
    %   Custom Add-on for Arduino to allow the Arduino library
    %   ArduinoNunchuk.h to communicate with MATLAB
    
    properties(Access = private, Constant = true)
        % CMD_IDs as defined in Nunchuk.h
        NUNCHUK_CREATE = hex2dec('00')
        NUNCHUK_UPDATE = hex2dec('01')
        NUNCHUK_INIT = hex2dec('02')
    end
    
    properties(Access = protected, Constant = true)
        % Add-on required fields
        LibraryName = 'Nunchuk/Nunchuk'
        DependentLibraries = {}
        LibraryHeaderFiles = 'ArduinoNunchuk/ArduinoNunchuk.h'
        CppHeaderFile = fullfile(arduinoio.FilePath(mfilename('fullpath')), 'src', 'Nunchuk.h')
        CppClassName = 'Nunchuk'
    end
    
    properties(Access = private)
        ResourceOwner = 'Nunchuk/Nunchuk';
        Pins = {'A4','A5'}; % Pins to the nunchuk wiring
    end
    
    properties(Access = public)
        % Store the controller fields
        analogX;
        analogY;
        accelX;
        accelY;
        accelZ;
        buttonZ;
        buttonC;
    end
    
    methods(Hidden, Access = public)
        function self = NunchukAddon(parentObject)
            %NUNCHUKADDON Construct an instance of this class
            %   NunchukAddon is a Custom Add-on to allow MATLAB to
            %   communicate with the Nunchuk.h file which communicates with
            %   the ArduinoNunchuk.h library.
            
            try
                p = inputParser;
                addParameter(p, 'DataPins', []);
            catch e
                throwAsCaller(e);
            end
            
            self.Parent = parentObject;
            % disp(self.Pins);
            
            count = getResourceCount(self.Parent, self.ResourceOwner);
            
            if count > 0
                error('You can only have one Nunchuk');
            end
            incrementResourceCount(self.Parent, self.ResourceOwner);
            createNunchuk(self);
        end
        
        function createNunchuk(self)
            %CREATENUNCHUK Create an instantiation of the Nunchuk.h class
            try
                cmdID = self.NUNCHUK_CREATE;
                
                configurePinResource(self.Parent, 'A4', self.ResourceOwner,'I2C');
                configurePinResource(self.Parent, 'A5', self.ResourceOwner,'I2C');
                
                % Send the requirested method to the board
                sendCommand(self, self.LibraryName, cmdID, []);
            catch e
                throwAsCaller(e);
            end
        end
    end
    
    methods(Access = public)
        function init(self)
            % Initialize the Nunchuk class
            cmdID = self.NUNCHUK_INIT;
            % Send the method request to the board
            sendCommand(self, self.LibraryName, cmdID, []);
        end
        function nunchukResults = update(self)
            % Update the Nunchuk class, retrieve new values and store them
            % in this object.
            cmdID = self.NUNCHUK_UPDATE;
            
            % Data is configured in bytes, so retrieve the different
            % individual bytes
            try
            out = sendCommand(self, self.LibraryName, cmdID, [], 0.1);
            self.analogX = 256 * out(1) + out(2);
            self.analogY = 256 * out(3) + out(4);
            self.accelX = 256 * out(5) + out(6);
            self.accelY = 256 * out(7) + out(8);
            self.accelZ = 256 * out(9) + out(10);
            self.buttonZ = out(11);
            self.buttonC = out(12);
            
            catch ME, 
                % Timeout will throw an error, so catch it and do nothing
                % disp(ME);
                % Do nothing with this error
            end
            % Return the values from the update function
            nunchukResults = [self.analogX self.analogY self.accelX self.accelY self.accelZ self.buttonZ self.buttonC];
        end
    end
end

