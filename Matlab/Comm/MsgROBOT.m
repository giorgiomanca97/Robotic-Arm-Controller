classdef MsgROBOT < Message
    properties (Access = private)
        TimeSampling_us (1,1) uint32;
    end

    
    % Constructor
    methods (Access = public)
        function obj = MsgROBOT(num)
            if(nargin < 1)
                num = 0;
            end
            
            obj@Message(Code.ROBOT, num);
        end
    end

    
    % Getters
    methods (Access = public)
        function count = getCount(obj)
            arguments
                obj (1,1) MsgROBOT;
            end

            count = obj.getNum() + 1;
        end

        function ts_us = getTimeSampling(obj)
            arguments
                obj (1,1) MsgROBOT;
            end

            ts_us = obj.TimeSampling_us;
        end
    end

    
    % Setters
    methods (Access = public)
        function res = setCount(obj, count)
            arguments 
                obj (1,1) MsgROBOT;
                count (1,1) {mustBeInteger}
            end
            
            res = obj.setNum(count - 1);
        end

        function res = setTimeSampling(obj, ts_us)
            arguments 
                obj (1,1) MsgROBOT;
                ts_us (1,1) {mustBeInteger}
            end
            
            obj.TimeSampling_us = uint32(ts_us);
            res = true;
        end
    end
    

    % Data Buffer payload
    methods (Access = protected)
        function dim = bsize_payload(obj)
            arguments
                obj (1,1) MsgROBOT;
            end

            dim = 4;
        end
        
        function res = parse_payload(obj, data)
            arguments
                obj (1,1) MsgROBOT;
                data (1,:) uint8;
            end

            if(length(data) ~= obj.bsize_payload())
                res = false;
                return;
            end

            obj.setTimeSampling(Message.bytesToValue(data(1:4), 'uint32'));
            
            res = true;
        end

        function data = bytes_payload(obj)
            arguments
                obj (1,1) MsgROBOT;
            end

            data = zeros([1, obj.bsize_payload()], 'uint8');

            data(1:4) = Message.valueToBytes(obj.getTimeSampling(), 'uint32');
        end
    end
end