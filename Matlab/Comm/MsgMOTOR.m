classdef MsgMOTOR < Message
    properties (Access = private)

    end

    
    % Constructor
    methods (Access = public)
        function obj = MsgMOTOR()
            obj@Message(Code.MOTOR);
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
                obj (1,1) MsgMOTOR;
            end


        end
        
        function res = parse_payload(obj, data)
            arguments
                obj (1,1) MsgMOTOR;
                data (1,:) uint8;
            end


        end

        function data = bytes_payload(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end


        end
    end
end