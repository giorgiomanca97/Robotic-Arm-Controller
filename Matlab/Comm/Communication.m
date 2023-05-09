classdef Communication
    properties (Access = public)
        Serial (1,1);
    end


    methods
        function obj = Communication(varargin)
            obj.Serial = serialport(varargin{:});
        end
    end


    methods
        function [res, hdr] = peek(obj, timeout_us)
            arguments
                obj (1,1) Communication;
                timeout_us (1,1) {mustBeInteger, mustBePositive};
            end
        end

        function [res, msg] = rcv(obj, timeout_us)
            arguments
                obj (1,1) Communication;
                timeout_us (1,1) {mustBeInteger, mustBePositive};
            end
        end
        
        function res = snd(obj, msg)
            arguments
                obj (1,1) Communication;
                msg (1,1) Message;
            end
        end
    end
end