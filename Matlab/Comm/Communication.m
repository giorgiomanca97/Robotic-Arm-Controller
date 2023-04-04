classdef Communication
    methods (Static)
        function data = snd_idle()

        end

        function data = snd_pwm()

        end

        function data = snd_ref()

        end

        function data = snd_robot()

        end

        function data = snd_motor()

        end

        function data = snd_pid()

        end

        function data = snd_ackc()

        end

        function data = snd_acks()

        end

        function data = snd_error()

        end

        function data = rcv_idle()

        end
    end

    methods (Static, Access = private)
        function snd_ctrl(obj, command, values)            
            bw = obj.Serial.NumBytesWritten;
            data = sndctrl(obj.N, command, values);
            %disp(dec2bin(data));
            obj.Serial.write(data, 'uint8');
            while((obj.Serial.NumBytesWritten - bw) < (2+obj.N))
            end
        end
        
        function [num, status, switches, values] = rcv_ctrl(obj)
            while(obj.Serial.NumBytesAvailable < (3+obj.N))
            end
            data = obj.Serial.read(3+obj.N, 'uint8');
            %disp(dec2bin(data));
            [num, status, switches, values] = rcvctrl(data);
            obj.Encs = obj.Encs + values;
        end
        
        function snd_setup(obj, num, values)
            bw = obj.Serial.NumBytesWritten;
            data = sndsetup(num, values);
            %disp(dec2bin(data));
            obj.Serial.write(data, 'uint8');
            while((obj.Serial.NumBytesWritten - bw) < 25)
            end
        end
        
        function result = rcv_setup(obj)
            while(obj.Serial.NumBytesAvailable < 1)
            end
            data = obj.Serial.read(1, 'uint8');
            %disp(dec2bin(data));
            result = rcvsetup(data);
        end
    end
end