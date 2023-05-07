classdef MsgMOTOR < Message
    properties (Access = private)

    end

    
    % Constructor
    methods (Access = public)
        function obj = MsgMOTOR()

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
        
        function res = parse_payload(obj, buffer)
            arguments
                obj (1,1) MsgMOTOR;
                buffer (1,:) uint8;
            end


        end

        function buffer = bytes_payload(obj)
            arguments
                obj (1,1) MsgMOTOR;
            end


        end
    end
end