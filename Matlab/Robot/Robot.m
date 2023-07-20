classdef Robot < serialport
    properties (Access = private)
        N;      % Robot size
        Ends;   % Robot endstops value
        Encs;   % Robot encoders value
        Refs;   % Target encoders value
    end
    
    
    % Constructor
    methods
        function obj = Robot(n, port, baudrate)
            if(n <= 0)
                error("Number of motors must be at least 1.");
            end
            
            if(n > 8)
                error("The protocol support up to 8 motors.");
            end
            
            obj = obj@serialport(port, baudrate);
            obj.flush();

            obj.N = uint8(n);
            obj.Ends = false([obj.N, 1]);
            obj.Encs = zeros([obj.N, 1], 'int64');
            obj.Refs = zeros([obj.N, 1], 'int64');
            
        end
    end
    
    
    % Getters
    methods (Access = public)
        function n = getSize(obj)
            n = obj.N;
        end
        
        function endstops = getEndstops(obj)
            endstops = obj.Ends;
        end

        function encoders = getEncoders(obj)
            encoders = obj.Encs;
        end
    end
    
    
    % Communication
    methods (Access = public)
        function res = ctrl_idle(obj)
            %snd
            %rcv
            obj.Encs = obj.Encs + delta;
            obj.Ends = switches;
            res = true;
        end

        function res = ctrl_pwm(obj, pwms)
            %snd
            %rcv
            obj.Encs = obj.Encs + delta;
            obj.Ends = switches;
            res = true;
        end

        function res = ctrl_ref(obj, encs)
            %snd
            %rcv
            obj.Encs = obj.Encs + delta;
            obj.Ends = switches;
            res = true;
        end

        function res = setup_robot(obj, timesampling)
            %snd
            %rcv
            obj.Encs = obj.Encs + delta;
            obj.Ends = switches;
            res = true;
        end

        function res = setup_motor(obj, index, spin_dir, enc_dir, enc)
            %snd
            %rcv
            obj.Encs = obj.Encs + delta;
            obj.Ends = switches;
            res = true;
        end

        function res = setup_pid(obj, index, div, kp, ki, kd, sat, pole)
            %snd
            %rcv
            obj.Encs = obj.Encs + delta;
            obj.Ends = switches;
            res = true;
        end        
    end
end
