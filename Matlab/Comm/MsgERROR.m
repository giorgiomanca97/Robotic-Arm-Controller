classdef MsgERROR < Message
    properties (Access = private)

    end

    
    % Constructor
    methods (Access = public)
        function obj = MsgERROR()

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
                obj (1,1) MsgERROR;
            end


        end
        
        function res = parse_payload(obj, buffer)
            arguments
                obj (1,1) MsgERROR;
                buffer (1,:) uint8;
            end


        end

        function buffer = bytes_payload(obj)
            arguments
                obj (1,1) MsgERROR;
            end


        end
    end
end