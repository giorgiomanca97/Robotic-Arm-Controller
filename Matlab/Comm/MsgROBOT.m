classdef MsgROBOT < Message
    properties (Access = private)

    end

    
    % Constructor
    methods (Access = public)
        function obj = MsgROBOT()
            obj@Message(Code.ROBOT);
        end
    end

    
    % Getters
    methods (Access = public)

    end

    
    % Setters
    methods (Access = public)

    end
    

    % Data Buffer payload
    methods (Access = protected)
        function dim = bsize_payload(obj)
            arguments
                obj (1,1) MsgROBOT;
            end


        end
        
        function res = parse_payload(obj, data)
            arguments
                obj (1,1) MsgROBOT;
                data (1,:) uint8;
            end


        end

        function data = bytes_payload(obj)
            arguments
                obj (1,1) MsgROBOT;
            end


        end
    end
end