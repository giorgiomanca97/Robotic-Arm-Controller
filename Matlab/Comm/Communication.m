classdef Communication < handle
    properties (Access = private)
        Serial (1,1);
        PeekByte (1,1) uint8;
        Peeked (1,1) logical;
    end


    methods
        function obj = Communication(varargin)
            obj.Serial = serialport(varargin{:});
            obj.PeekByte = uint8(0);
            obj.Peeked = false;
        end
    end


    methods
        function [res, hdr] = peek(obj, timeout_us)
            arguments
                obj (1,1) Communication;
                timeout_us (1,1) {mustBeInteger, mustBeNonnegative} = 0;
            end
            
            time = tic();

            if(~obj.Peeked)
                if(timeout_us > 0)
                    while(obj.Serial.NumBytesAvailable < 1 && toc(time) * 1e6 <= timeout_us)
                    end
                end
                if(obj.Serial.NumBytesAvailable > 0)
                    obj.PeekByte = obj.Serial.read(1, 'uint8');
                    obj.Peeked = true;
                end
            end
            
            hdr = Header();
            if(obj.Peeked) 
                hdr.parse(obj.PeekByte);
                res = true;
            else
                res = false;
            end            
        end
        
        
        function [res, msg] = rcv(obj, timeout_us)
            arguments
                obj (1,1) Communication;
                timeout_us (1,1) {mustBeInteger, mustBeNonnegative} = 0;
            end
            
            time = tic();
            
            if(obj.Peeked)
                hdr = Header();
                hdr.parse(obj.PeekByte); 
            else
                [res, hdr] = obj.peek(timeout_us);
                if(~res)
                    msg = MsgERROR(hdr.getNum());
                    return;
                end
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
                while((obj.Serial.NumBytesAvailable + 1) < msg.bsize() && toc(time) * 1e6 <= timeout_us)
                end
            end

            if((obj.Serial.NumBytesAvailable + 1) >= msg.bsize())
                if (msg.bsize() > 1)
                    body = obj.Serial.read(msg.bsize()-1, 'uint8');
                else
                    body = zeros([1,0], 'uint8');
                end
                res = msg.parse([obj.PeekByte, body]);
                obj.Peeked = false;
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
            
            obj.Peeked = false;
            obj.Serial.flush();
        end
    end
end