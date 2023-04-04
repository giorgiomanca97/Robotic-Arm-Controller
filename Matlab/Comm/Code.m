classdef Code < uint8
    enumeration
        IDLE (0)
        PWM  (1)
        REF  (2)
        ROBOT(16)
        MOTOR(17)
        PID  (18)
        ACKC (24)
        ACKS (25)
        ERROR(31)
    end

    methods (Static)
        function obj = from(value)
            if(nargin < 1)
                value = Code.IDLE;
            end

            switch(uint8(value))
                case Code.IDLE
                    obj = Code.IDLE;
                case Code.PWM
                    obj = Code.PWM;
                case Code.REF
                    obj = Code.REF;
                case Code.ROBOT
                    obj = Code.ROBOT;
                case Code.MOTOR
                    obj = Code.MOTOR;
                case Code.PID
                    obj = Code.PID;
                case Code.ACKC
                    obj = Code.ACKC;
                case Code.ACKS
                    obj = Code.ACKS;
                case Code.ERROR
                    obj = Code.ERROR;
                otherwise
                    obj = Code.ERROR;
            end
        end
    end
end