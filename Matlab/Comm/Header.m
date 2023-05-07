classdef Header < handle
    properties (Access = private)
        Cod (1,1) Code;
        Num (1,1) uint8;
    end
    
    
    % Constructor
    methods (Access = public)
        function obj = Header()

        end
    end
    
    
    % Getters
    methods (Access = public)
        function code = getCode(obj)
            arguments
                obj (1,1) Header;
            end

            code = obj.Cod;
        end

        function num = getNum(obj)
            arguments
                obj (1,1) Header;
            end

            num = obj.Num;
        end
    end
    
    
    % Setters
    methods (Access = public)
        function res = setCode(obj, code)
            arguments
                obj (1,1) Header;
                code (1,1) Code;
            end
            
            obj.Cod = code;
            res = true;
        end

        function res = setNum(obj, num)
            arguments
                obj (1,1) Header;
                num (1,1) {mustBeInteger};
            end
            
            if(num >= 0 && num <= 7)
                obj.Num = uint8(num);
                res = true;
            else
                res = false;
            end
        end
    end


    % Data Buffer
    methods (Access = public)
        function dim = bsize(obj)
            arguments
                obj (1,1) Header;
            end

            dim = 1;
        end

        function res = parse(obj, buffer)
            arguments
                obj (1,1) Header;
                buffer (1,:) uint8;
            end

            [code, res] = Code.convert(bitshift(buffer(1), -3, 'uint8'));
            if(res)
                obj.Cod = code;
                obj.Num = bitand(buffer(1), 0b00000111, 'uint8');
            end
        end

        function buffer = bytes(obj)
            arguments
                obj (1,1) Header;
            end

            buffer = zeros([1,1], 'uint8');
            buffer(1) = bitor(bitshift(obj.Cod, 3, 'uint8'), obj.Num, 'uint8');
        end
    end
end