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
                obj (1,1) Code;
            end

            code = obj.Hdr.getCode();
        end

        function num = getNum(obj)
            arguments
                obj (1,1) Code;
            end

            num = obj.Hdr.getNum();
        end
    end

    
    % Setters
    methods (Access = public)
        function res = setNum(obj, num)
            arguments
                obj (1,1) Code;
                num (1,1) {mustBeInteger};
            end
            
            res = obj.Hdr.setNum(num);
        end
    end
    

    % Data Buffer
    methods (Access = public)
        function dim = bsize(obj)
            arguments
                obj (1,1) Code;
            end

            dim = obj.bsize_header() + obj.bsize_payload();
        end

        function res = parse(obj, buffer)
            arguments
                obj (1,1) Code;
                buffer (1,:) uint8;
            end
            
            res = obj.parse_header(buffer);
            if(~res)
                return;
            else
                res = obj.parse_payload(buffer);
            end
        end

        function buffer = bytes(obj)
            arguments
                obj (1,1) Code;
            end
            
            buffer = zeros([1, obj.bsize()], 'uint8');
            buffer(:) = [obj.bytes_header(), obj.bytes_payload()];
        end
    end


    % Data Buffer payload
    methods (Access = protected, Abstract)
        dim = bsize_payload(obj)
        
        res = parse_payload(obj, buffer)

        buffer = bytes_payload(obj)
    end
    

    % Data Buffer header
    methods (Access = private)
        function dim = bsize_header(obj)
            arguments
                obj (1,1) Code;
            end

            dim = obj.Hdr.bsize();
        end

        function res = parse_header(obj, buffer)
            arguments
                obj (1,1) Code;
                buffer (1,:) uint8;
            end
            
            [code, res] = Code.convert(bitshift(buffer(1), -3, 'uint8'));
            if(res && code == obj.getCode())
                obj.Hdr.setNum(bitand(buffer(1), 0b00000111, 'uint8'));
            else
                res = false;
            end
        end

        function buffer = bytes_header(obj)
            arguments
                obj (1,1) Code;
            end
            
            buffer = obj.Hdr.bytes();
        end
    end
end