# **Serial Communication**

Serial communication protocol for DC motors control by a microcontroller (Arduino, Raspberry, STM32, ...), supporting the following features:
- Support up to 8 motors (N = numbers of motors)
- The companion PC specify command/operation-mode to the microcontroller.
- The board communicate its state and whether the motors reached theirs endstops.
- Use 8+1 bits for pwms and delta-encoders transmission (the extra bit is for value's sign).
- Allow to setup microcontroller on-board PIDs/Parameters.


The messages have the following format:

|  Byte | Description                                                                           |
| ----- | ------------------------------------------------------------------------------------- |
|   1   | **Header**: carry command/status codes and motor number/selection.                    |
|  2-X  | **Payload**: carry data coherent to the command/status specified in the header byte.  |

There are two types of messages:
- **Control messages**: used during control.
- **Setup messages**: used to set on-board controllers parameters.

<br><br>

## **Sending (From companion PC to Microcontroller)**

<br>

The companion PC sends as **control messages** of 2+N bytes with the following format:

|  Byte | Description                                                                           |
| ----- | ------------------------------------------------------------------------------------- |
|   1   | Bit [1-3] number of motors (N-1).<br> Bit [4-8] control command to board.             |
|   2   | Bit-mask for motors directions [87654321].<br> Bit value: 0 = positive, 1 = negative. |
|   3   | unsigned PWM/delta-encoder value (from 0 to 255) for motor 1.                         |
|  ...  | ...                                                                                   |
|  N+2  | unsigned PWM/delta-encoder value (from 0 to 255) for motor N.                         |

<br>

The companion PC sends as **setup messages** of 25 bytes with the following format:

|  Byte | Description                                                                           |
| ----- | ------------------------------------------------------------------------------------- |
|   1   | Bit [1-3] motor index [0, N-1] selected. <br> Bit [4-8] setup command.                |
|  2-5  | Float (little-endian) for encoder error divider.                                      |
|  6-9  | Float (little-endian) for PID proportional coefficient.                               |
| 10-13 | Float (little-endian) for PID integral coefficient.                                   |
| 14-17 | Float (little-endian) for PID derivative coefficient.                                 |
| 18-21 | Float (little-endian) for PID dirty derivative pole.                                  |
| 20-25 | Float (little-endian) for PID integral saturation.                                    |

<br>

The PID values are expressed in continuos time domain (discretization made on-board according to internal time sampling).\
Float are expressed with 4 bytes with little-endian notation, which mean from least to most significant byte, that is:
~~~
float 32 bits:  D8 D7 D6 D5 D4 D3 D2 D1 | C8 C7 C6 C5 C4 C3 C2 C1 | B8 B7 B6 B5 B4 B3 B2 B1 | A8 A7 A6 A5 A4 A3 A2 A1

1st byte sent:  A8 A7 A6 A5 A4 A3 A2 A1
2nd byte sent:  B8 B7 B6 B5 B4 B3 B2 B1
3rd byte sent:  C8 C7 C6 C5 C4 C3 C2 C1
4th byte sent:  D8 D7 D6 D5 D4 D3 D2 D1

bytes sent:     A8 A7 A6 A5 A4 A3 A2 A1 | B8 B7 B6 B5 B4 B3 B2 B1 | C8 C7 C6 C5 C4 C3 C2 C1 | D8 D7 D6 D5 D4 D3 D2 D1
~~~

<br>

**Command values** (last 5 bits of header byte) can be:

| Value |  Bits | Description                                                                   |
| ----- | ----- | ----------------------------------------------------------------------------- |
|   0   | 00000 | Idle: set all motors speed to zero.                                           |
|   1   | 00001 | DAQ mode: directly assign PWMs values.                                        |
|   2   | 00010 | PID mode: assign desired encoders values.                                     |
|   3   | 00011 | Setup: sending parameters for PID specified in header.                        |
| other | xxxxx | Unused.                                                                       |

<br>

## **Receiving (From Microcontroller to Companion PC)**

<br>

The microcontroller sends as **control response** of 3+N bytes with the following format:

| Byte | Description                                                                            |
| ---- | -------------------------------------------------------------------------------------- |
|   1  | Bit [1-3] number of motors (N).<br> Bit [4-8] board's status.                          |
|   2  | Bit-mask for motors end-stop state [87654321].<br> Bit value: 0 = false, 1 = true      |
|   3  | Bit-mask for encoders directions [87654321].<br> Bit value: 0 = positive, 1 = negative.|
|   4  | unsigned delta-encoder value (from 0 to 255) of motor 1.                               |
|  ... | ...                                                                                    |
|  N+3 | unsigned delta-encoder value (from 0 to 255) of motor N.                               |

<br>

The microcontroller sends as **setup response** of 1 byte with the following format:

|  Byte | Description                                                                           |
| ----- | ------------------------------------------------------------------------------------- |
|   1   | Bit [1-3] set to zero. <br> Bit [4-8] setup status (as ACK).                          |

<br>

**Status values** (last 5 bits of header byte) can be:

| Value |  Bits | Description                                                                   |
| ----- | ----- | ----------------------------------------------------------------------------- |
|   0   | 00000 | Idle: all motors speed setted to zero.                                        |
|   1   | 00001 | DAQ mode: operating as a DAQ using pwms received.                             |
|   2   | 00010 | PID mode: using PID controllers to follow encoder set-points.                 |
|   3   | 00011 | Setup: PID parameters received.                                               |
| other | xxxxx | Unused.                                                                       |

<br>

## **Protocol**

<br>

The communication is always started by the Companion PC, while the Microcontroller always reply to a received message.\
Conversely, the Microcontroller never send a message (namely a reply) if not triggered by a message from the Companion PC.\
The exchange of message is timed by the microcontroller only during DAQ and PID mode.\
Then overall the communication consists of alternating messages from the Companion PC and replies from the Microcontroller, always started by the first one and eventually timed by the second one.