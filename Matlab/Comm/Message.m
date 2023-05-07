classdef Message < handle
    properties (Access = private)
        Hdr (1,1) Header;
    end
    

    % Constructor
    methods (Access = public)
        function obj = Message(code)
            arguments
                code (1,1) Code;
            end
            
            obj.Hdr.setCode(code);
        end
    end
    
    
    % Getters
    methods (Access = public)
        function code = getCode(obj)
            arguments
                obj (1,1) Message;
            end

            code = obj.Hdr.getCode();
        end

        function num = getNum(obj)
            arguments
                obj (1,1) Message;
            end

            num = obj.Hdr.getNum();
        end
    end

    
    % Setters
    methods (Access = public)
        function res = setNum(obj, num)
            arguments
                obj (1,1) Message;
                num (1,1) {mustBeInteger};
            end
            
            res = obj.Hdr.setNum(num);
        end
    end
    

    % Data Buffer
    methods (Access = public)
        function dim = bsize(obj)
            arguments
                obj (1,1) Message;
            end

            dim = obj.bsize_header() + obj.bsize_payload();
        end

        function res = parse(obj, data)
            arguments
                obj (1,1) Message;
                data (1,:) uint8;
            end
            
            if(length(data) ~= obj.bsize())
                res = false;
                return;
            end
            
            if(obj.parse_header(data(1:obj.bsize_header())))
                res = obj.parse_payload(data(1+obj.bsize_header():end));
            else
                res = false;
            end
        end

        function data = bytes(obj)
            arguments
                obj (1,1) Message;
            end
            
            data = zeros([1, obj.bsize()], 'uint8');
            data(:) = [obj.bytes_header(), obj.bytes_payload()];
        end
    end


    % Data Buffer payload
    methods (Access = protected, Abstract)
        dim = bsize_payload(obj)
        
        res = parse_payload(obj, data)

        data = bytes_payload(obj)
    end
    

    % Data Buffer header
    methods (Access = private)
        function dim = bsize_header(obj)
            arguments
                obj (1,1) Message;
            end

            dim = obj.Hdr.bsize();
        end

        function res = parse_header(obj, data)
            arguments
                obj (1,1) Message;
                data (1,:) uint8;
            end
            
            if(length(data) ~= obj.bsize_header())
                res = false;
                return;
            end
            
            tmp = Header();
            res = tmp.parse(data);
            if(res && code == tmp.getCode())
                res = obj.Hdr.parse(data);
            else
                res = false;
            end
        end

        function data = bytes_header(obj)
            arguments
                obj (1,1) Message;
            end
            
            data = obj.Hdr.bytes();
        end
    end
end