classdef MsgACKC < Message
    properties (Access = private)

    end

    
    % Constructor
    methods (Access = public)
        function obj = MsgACKC()

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
                obj (1,1) MsgACKC;
            end


        end
        
        function res = parse_payload(obj, buffer)
            arguments
                obj (1,1) MsgACKC;
                buffer (1,:) uint8;
            end


        end

        function buffer = bytes_payload(obj)
            arguments
                obj (1,1) MsgACKC;
            end


        end
    end
end