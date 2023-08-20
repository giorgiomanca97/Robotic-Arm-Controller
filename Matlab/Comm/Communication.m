classdef Communication
    properties (Access = public)
        Serial (1,1);
        PeekByte (1,1) uint8;
        Peeked (1,1) logical;
    end


    methods
        function obj = Communication(varargin)
            obj.Serial = serialport(varargin{:});
            obj.PeekByte(1) = uint8(0);
            obj.Peeked(1) = false;
        end
    end


    methods
        function [res, hdr] = peek(obj, timeout_us)
            arguments
                obj (1,1) Communication;
                timeout_us (1,1) {mustBeInteger, mustBeNonnegative};
            end
            
            time = tic();

            if(~obj.Peeked)
                if(timeout_us > 0)
                    while(obj.Serial.NumBytesAvailable < 1 && toc(time) * 1e6 <= delta)
                    end
                end
                if(obj.Serial.NumBytesAvailable > 0)
                    obj.PeekByte(1) = obj.Serial.read(1, 'uint8');
                    obj.Peeked(1) = true;
                end
            end
            
            hdr = Header();
            res = obj.Peeked;
            if(obj.Peeked) 
                hdr.parse(obj.PeekByte);
            end
        end

        function [res, msg] = rcv(obj, timeout_us)
            arguments
                obj (1,1) Communication;
                timeout_us (1,1) {mustBeInteger, mustBeNonnegative};
            end
            
            time = tic();
            
            if(~obj.Peeked)
                [res, hdr] = obj.peek(timeout_us);
                if(~res)
                    msg = MsgERROR(hdr.getNum());
                    return;
                end
            else
                hdr = Header();
                hdr.parse(obj.PeekByte);
            end

            switch(hdr.getCode())
                case Code.IDLE
                    msg = MsgIDLE();
                case Code.PWM
                    msg = MsgPWM();
                case Code.REF
                    msg = MsgREF();
                case Code.ROBOT
                    msg = MsgROBOT();
                case Code.MOTOR
                    msg = MsgMOTOR();
                case Code.PID
                    msg = MsgPID();
                case Code.ACKC
                    msg = MsgACKC();
                case Code.ACKS
                    msg = MsgACKS();
                case Code.ERROR
                    msg = MsgERROR();
            end

            msg.setNum(hdr.getNum());

            if(timeout_us > 0)
                while(obj.Serial.NumBytesAvailable < msg.bsize() && toc(time) * 1e6 <= delta)
                end
            end

            if(obj.Serial.NumBytesAvailable >= msg.bsize())
                res = msg.parse(obj.Serial.read(msg.bsize(), 'uint8'));
                obj.Peeked(1) = false;
            else
                res = false;
            end
        end
        
        function res = snd(obj, msg)
            arguments
                obj (1,1) Communication;
                msg (1,1) Message;
            end
            
            w = obj.Serial.NumBytesWritten;
            obj.Serial.write(msg.bytes(), 'uint8');
            res = (obj.Serial.NumBytesWritten - w) == msg.bsize();
        end

        function flush(obj)
            arguments
                obj (1,1) Communication;
            end
            
            obj.Peeked(1) = false;
            obj.Serial.flush();
        end
    end
end