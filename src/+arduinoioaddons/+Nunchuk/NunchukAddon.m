classdef NunchukAddon < matlabshared.addon.LibraryBase
    %NUNCHUKADDON Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access = private, Constant = true)
        NUNCHUK_CREATE = hex2dec('00')
        NUNCHUK_UPDATE = hex2dec('01')
        NUNCHUK_INIT = hex2dec('02')
    end
    
    properties(Access = protected, Constant = true)
        LibraryName = 'Nunchuk/Nunchuk'
        DependentLibraries = {}
        LibraryHeaderFiles = 'ArduinoNunchuk/ArduinoNunchuk.h'
        CppHeaderFile = fullfile(arduinoio.FilePath(mfilename('fullpath')), 'src', 'Nunchuk.h')
        CppClassName = 'Nunchuk'
    end
    
    properties(Access = private)
        ResourceOwner = 'Nunchuk/Nunchuk';
        Pins = {'A4','A5'};
    end
    
    methods(Hidden, Access = public)
        function self = NunchukAddon(parentObject)
            %NUNCHUKADDON Construct an instance of this class
            %   Detailed explanation goes here
            
            try
                p = inputParser;
                addParameter(p, 'DataPins', []);
            catch e
                throwAsCaller(e);
            end
            
            self.Parent = parentObject;
            disp(self.Pins);
            
            
            count = getResourceCount(self.Parent, self.ResourceOwner);
            
            if count > 0
                error('You can only have one Nunchuk');
            end
            incrementResourceCount(self.Parent, self.ResourceOwner);
            createNunchuk(self);
        end
        
        function createNunchuk(self)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            try
                cmdID = self.NUNCHUK_CREATE;
                
                configurePinResource(self.Parent, 'A4', self.ResourceOwner,'I2C');
                configurePinResource(self.Parent, 'A5', self.ResourceOwner,'I2C');
                
                sendCommand(self, self.LibraryName, cmdID, []);
            catch e
                throwAsCaller(e);
            end
        end
    end
    
    methods(Access = public)
        function init(self)
           cmdID = self.NUNCHUK_INIT;
           disp("Calling Init");
           sendCommand(self, self.LibraryName, cmdID, []);
        end
        function update(self)
            cmdID = self.NUNCHUK_UPDATE;
            
            sendCommand(self, self.LibraryName, cmdID, []);
        end
    end
end

