classdef HelloWorld < matlabshared.addon.LibraryBase
    properties (Access = private, Constant = true)
        READ_COMMAND = hex2dec('01');
    end
    
    properties (Access = protected, Constant = true)
        LibraryName = 'HelloWorld/HelloWorld';
        DependentLibraries = {}
        LibraryHeaderFiles = {}
        CppHeaderFile = fullfile(arduinoio.FilePath(mfilename('fullpath')), 'src', 'HelloWorld.h')
        CppClassName = 'HelloWorld'
    end
    
    methods
        function self = HelloWorld(parent)
            self.Parent = parent;
        end
        
        function out = read(obj)
            cmdID = obj.READ_COMMAND;
            inputs = [];
            output = sendCommand(obj, obj.LibraryName, cmdID, inputs);
            out = char(output');
        end
    end
end