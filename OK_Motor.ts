
enum motor {
    //% block="A"
    M1,
    //% block="B"
    M2,
    //% block="AB"
    M12
}

enum motorDIR {
    //% block="Forward"
    Forward,
    //% block="Backward"
    Backward
}

enum turn {
    //% block="left"
    Left,
    //% block="right"
    Right
}

enum iKB1Motor {
    //% block="Forward \u21c8"
    Forward,
    //% block="Backward \u21ca"
    Backward
}

enum iKB1Turn {
    //% block="Left \u27f5"
    Left,
    //% block="Right \u27f6"
    Right
}

/**
  * Enumeration of SpinMotor.
  */
enum iKB1Spin {
    //% block="Left \u21f5"
    Left,
    //% block="Right \u21c5"
    Right
}


enum Servo {
    //% block="P0"
    Servo0,
    //% block="P1"
    Servo1,
    //% block="P2"
    Servo2,
    //% block="P3"
    Servo3,
    //% block="P4"
    Servo4,
    //% block="P10"
    Servo10,
    //% block="P5"
    Servo5,
    //% block="P6"
    Servo6,
    //% block="P7"
    Servo7,
    //% block="P8"
    Servo8,
    //% block="P9"
    Servo9,
    //% block="P11"
    Servo11,
    //% block="P12"
    Servo12
}


/**
 * Custom blocks
 */
//% weight=10 color=#003399 weight=10 icon="\uf2a3"
namespace OK_Motor {
    export enum analogPort {
        P0,
        P1,
        P2,
        P3,
        P4,
        P10
    }
    /**MotorON          Control motor channel direction and speed.   
    * @param Speed  	  Percent of motor speed, eg: 50
    */
    //% blockId="Motor_MotorON" block="motor %motorSEL | direction %motorDIR | speed %Speed"
    //% Speed.min=0 Speed.max=100
    //% weight=91
    export function Motor(Channel: motor, Direction: motorDIR, Speed: number): void {
        let motorspeed = pins.map(Speed, 0, 100, 1023, 0)

        if (Channel == motor.M1 && Direction == motorDIR.Forward) {
            pins.analogWritePin(AnalogPin.P14, motorspeed)
            pins.digitalWritePin(DigitalPin.P13, 1)
        }
        else if (Channel == motor.M2 && Direction == motorDIR.Forward) {
            pins.analogWritePin(AnalogPin.P16, motorspeed)
            pins.digitalWritePin(DigitalPin.P15, 1)
        }
        else if (Channel == motor.M1 && Direction == motorDIR.Backward) {
            pins.analogWritePin(AnalogPin.P13, motorspeed)
            pins.digitalWritePin(DigitalPin.P14, 1)
        }
        else if (Channel == motor.M2 && Direction == motorDIR.Backward) {
            pins.analogWritePin(AnalogPin.P15, motorspeed)
            pins.digitalWritePin(DigitalPin.P16, 1)
        }
        else if (Channel == motor.M12 && Direction == motorDIR.Forward) {
            pins.analogWritePin(AnalogPin.P14, motorspeed)
            pins.digitalWritePin(DigitalPin.P13, 1)
            pins.analogWritePin(AnalogPin.P16, motorspeed)
            pins.digitalWritePin(DigitalPin.P15, 1)
        }
        else if (Channel == motor.M12 && Direction == motorDIR.Backward) {
            pins.analogWritePin(AnalogPin.P13, motorspeed)
            pins.digitalWritePin(DigitalPin.P14, 1)
            pins.analogWritePin(AnalogPin.P15, motorspeed)
            pins.digitalWritePin(DigitalPin.P16, 1)
        }
    }

    /**MotorAB  Control motor AB with direction and motor speed, separate A, B.   
      * @param speedA   Percent of motor speed A, eg: 50
      * @param speedB   Percent of motor speed B, eg: 50
      */
    //% blockId="Motor_MotorAB" block="motor AB direction %motorDIR |speed A %speedA |speed B %speedB"
    //% speedA.min=0 speedA.max=100
    //% speedB.min=0 speedB.max=100
    //% weight=90
    export function MotorAB(Direction: motorDIR, speedA: number, speedB: number): void {
        let motorspeedA = pins.map(speedA, 0, 100, 1023, 0)
        let motorspeedB = pins.map(speedB, 0, 100, 1023, 0)

        if (Direction == motorDIR.Forward) {
            pins.analogWritePin(AnalogPin.P14, motorspeedA)
            pins.digitalWritePin(DigitalPin.P13, 1)
            pins.analogWritePin(AnalogPin.P16, motorspeedB)
            pins.digitalWritePin(DigitalPin.P15, 1)
        }
        if (Direction == motorDIR.Backward) {
            pins.analogWritePin(AnalogPin.P13, motorspeedA)
            pins.digitalWritePin(DigitalPin.P14, 1)
            pins.analogWritePin(AnalogPin.P15, motorspeedB)
            pins.digitalWritePin(DigitalPin.P16, 1)
        }
    }

    /**
     * Turn off the motor
     * @param motor   Which motor to turn off
     */
    //% blockId="Motor_Stop" block="motor %motorSEL | stop"
    //% weight=60
    export function motorStop(Channel: motor): void {
        if (Channel == motor.M12) {
            pins.digitalWritePin(DigitalPin.P13, 1)
            pins.digitalWritePin(DigitalPin.P14, 1)
            pins.digitalWritePin(DigitalPin.P15, 1)
            pins.digitalWritePin(DigitalPin.P16, 1)
        }
        else if (Channel == motor.M1) {
            pins.digitalWritePin(DigitalPin.P13, 1)
            pins.digitalWritePin(DigitalPin.P14, 1)
        }
        else if (Channel == motor.M2) {
            pins.digitalWritePin(DigitalPin.P15, 1)
            pins.digitalWritePin(DigitalPin.P16, 1)
        }
    }

    /**
     * Control motor for linefollow or turn direction of robot.
     * @param turnDIR      Turn Left or Right
     * @param speedturn    Motor speed; eg: 40
    */
    //% blockId="Motor_Turn" block="turn %Turn | speed %speedturn"
    //% speedturn.min=0 speedturn.max=100
    //% weight=70
    export function Turn(turnDIR: turn, speedturn: number): void {
        let motorspeedturn = pins.map(speedturn, 0, 100, 1023, 0)
        if (turnDIR == turn.Left) {
            pins.digitalWritePin(DigitalPin.P13, 0)
            pins.digitalWritePin(DigitalPin.P14, 0)
            pins.analogWritePin(AnalogPin.P16, motorspeedturn)
            pins.digitalWritePin(DigitalPin.P15, 1)
        }
        if (turnDIR == turn.Right) {
            pins.analogWritePin(AnalogPin.P14, motorspeedturn)
            pins.digitalWritePin(DigitalPin.P13, 1)
            pins.digitalWritePin(DigitalPin.P15, 0)
            pins.digitalWritePin(DigitalPin.P16, 0)
        }
    }

    /**
     * Execute dual motor to rotate Left and Right, non stop for use linefollow mode.
     * @param rotateLINE     	rotate robot direction.
     * @param speedline     	motor speed; eg: 50
     */
    //% blockId="Motor_Spin"  block="Spin %Turn |speed %speedline"
    //% speedline.min=0 speedline.max=100
    //% weight=80
    export function Spin(rotateLINE: turn, speedline: number): void {
        let motorspeedline = pins.map(speedline, 0, 100, 1023, 0)
        if (rotateLINE == turn.Left) {
            pins.analogWritePin(AnalogPin.P13, motorspeedline)
            pins.digitalWritePin(DigitalPin.P14, 1)
            pins.analogWritePin(AnalogPin.P16, motorspeedline)
            pins.digitalWritePin(DigitalPin.P15, 1)
        }
        if (rotateLINE == turn.Right) {
            pins.analogWritePin(AnalogPin.P14, motorspeedline)
            pins.digitalWritePin(DigitalPin.P13, 1)
            pins.analogWritePin(AnalogPin.P15, motorspeedline)
            pins.digitalWritePin(DigitalPin.P16, 1)
        }
    }


    /**
    * Control Servo P0 to P12 degree 0 - 180 degree 
    * @param Degree   Servo degree 0-180, eg: 90
    */

    //% blockId="Servo_ServoRun" block="Servo %Servo|degree %Degree"
    //% Degree.min=0 Degree.max=180
    //% color=#3333FF
    //% weight=50
    export function ServoRun(ServoSelect: Servo, Degree: number): void {
        if (ServoSelect == Servo.Servo0) {
            pins.servoWritePin(AnalogPin.P0, Degree)
        }
        if (ServoSelect == Servo.Servo1) {
            pins.servoWritePin(AnalogPin.P1, Degree)
        }
        if (ServoSelect == Servo.Servo2) {
            pins.servoWritePin(AnalogPin.P2, Degree)
        }
        if (ServoSelect == Servo.Servo3) {
            pins.servoWritePin(AnalogPin.P3, Degree)
        }
        if (ServoSelect == Servo.Servo4) {
            pins.servoWritePin(AnalogPin.P4, Degree)
        }
        if (ServoSelect == Servo.Servo10) {
            pins.servoWritePin(AnalogPin.P10, Degree)
        }
        if (ServoSelect == Servo.Servo5) {
            pins.servoWritePin(AnalogPin.P5, Degree)
        }
        if (ServoSelect == Servo.Servo6) {
            pins.servoWritePin(AnalogPin.P6, Degree)
        }
        if (ServoSelect == Servo.Servo7) {
            pins.servoWritePin(AnalogPin.P7, Degree)
        }
        if (ServoSelect == Servo.Servo8) {
            pins.servoWritePin(AnalogPin.P8, Degree)
        }
        if (ServoSelect == Servo.Servo9) {
            pins.servoWritePin(AnalogPin.P9, Degree)
        }
        if (ServoSelect == Servo.Servo11) {
            pins.servoWritePin(AnalogPin.P11, Degree)
        }
        if (ServoSelect == Servo.Servo12) {
            pins.servoWritePin(AnalogPin.P12, Degree)
        }
    }

    /**
     * Control Servo P0 to P12 set to stop
     */
    //% blockId="MyServo_ServoStop" block="Servo stop %Servo"
    //% color=#3333FF
    //% weight=40
    export function ServoStop(ServoSelect: Servo): void {
        if (ServoSelect == Servo.Servo0) {
            pins.servoSetPulse(AnalogPin.P0, 0)
        }
        if (ServoSelect == Servo.Servo1) {
            pins.servoSetPulse(AnalogPin.P1, 0)
        }
        if (ServoSelect == Servo.Servo2) {
            pins.servoSetPulse(AnalogPin.P2, 0)
        }
        if (ServoSelect == Servo.Servo3) {
            pins.servoSetPulse(AnalogPin.P3, 0)
        }
        if (ServoSelect == Servo.Servo4) {
            pins.servoSetPulse(AnalogPin.P4, 0)
        }
        if (ServoSelect == Servo.Servo10) {
            pins.servoSetPulse(AnalogPin.P10, 0)
        }
        if (ServoSelect == Servo.Servo5) {
            pins.servoSetPulse(AnalogPin.P5, 0)
        }
        if (ServoSelect == Servo.Servo6) {
            pins.servoSetPulse(AnalogPin.P6, 0)
        }
        if (ServoSelect == Servo.Servo7) {
            pins.servoSetPulse(AnalogPin.P7, 0)
        }
        if (ServoSelect == Servo.Servo8) {
            pins.servoSetPulse(AnalogPin.P8, 0)
        }
        if (ServoSelect == Servo.Servo9) {
            pins.servoSetPulse(AnalogPin.P9, 0)
        }
        if (ServoSelect == Servo.Servo11) {
            pins.servoSetPulse(AnalogPin.P11, 0)
        }
        if (ServoSelect == Servo.Servo12) {
            pins.servoSetPulse(AnalogPin.P12, 0)
        }
    }

    /**
     * Execute dual motor to Forward non stop for use linefollow mode.
     * @param rotateLINE     	rotate robot direction.
     * @param speedline     	motor speed; eg: 50
     * @param speeddrift     	motor speed; eg: 50
     * @param Ref     	motor speed; eg: 500
     */
    //% blockId="TrackLine" block="Track 2 sensors|Sensor Left%analogPort| Sensor Right%analogPort|Reference%Ref|Speed%speedline|SpeedDrift%speeddrift"
    //% speedline.min=0 speedline.max=100
    //% speeddrift.min=0 speeddrift.max=100
    //% weight=39
    export function track(PortL: analogPort, PortR: analogPort, Ref: number, speedline: number, speeddrift: number): void {
        let motorspeedline = pins.map(speedline, 0, 100, 1023, 0)
        let loop = true
        while(loop){
            if (PortL > Ref && PortR > Ref) {
                pins.digitalWritePin(DigitalPin.P13, 1)
                pins.analogWritePin(AnalogPin.P14, speedline)
                pins.digitalWritePin(DigitalPin.P15, 1)
                pins.analogWritePin(AnalogPin.P16, speedline)
            }
            else if (PortL < Ref && PortR > Ref) {
                pins.analogWritePin(AnalogPin.P13, speeddrift)
                pins.digitalWritePin(DigitalPin.P14, 1)
                pins.digitalWritePin(DigitalPin.P15, 1)
                pins.analogWritePin(AnalogPin.P16, speeddrift)  
            }
            else if (PortL > Ref && PortR < Ref) {
                pins.digitalWritePin(DigitalPin.P13, 1)
                pins.analogWritePin(AnalogPin.P14, speeddrift)
                pins.analogWritePin(AnalogPin.P15, speeddrift)
                pins.digitalWritePin(DigitalPin.P16, 1)
            }
            else if (PortL < Ref && PortR < Ref) {loop = false}
        }
       
    }

    /**
     * Execute dual motor to rotate Left and Right, non stop for use linefollow mode.
     * @param rotateLINE     	rotate robot direction.
     * @param speedline     	motor speed; eg: 50
     * @param Ref     	motor speed; eg: 500
     */
    //% blockId="Motor_turn90" block="Spin%Turn 90 Degrees|Speed %speedline|Port%analogPort|Reference%number"
    //% speedline.min=0 speedline.max=100
    //% weight=38
    export function Spin90(rotateLINE: turn, speedline: number, Port: analogPort, Ref: number): void {
        let motorspeedline = pins.map(speedline, 0, 100, 1023, 0)
        pins.digitalWritePin(DigitalPin.P13, 1)
        pins.digitalWritePin(DigitalPin.P14, 1)
        pins.digitalWritePin(DigitalPin.P15, 1)
        pins.digitalWritePin(DigitalPin.P16, 1)
        basic.pause(100)
        if (rotateLINE == turn.Left) {
            pins.analogWritePin(AnalogPin.P13, motorspeedline)
            pins.digitalWritePin(DigitalPin.P14, 1)
            pins.analogWritePin(AnalogPin.P16, motorspeedline)
            pins.digitalWritePin(DigitalPin.P15, 1)
            while (Port < Ref) {
                pins.analogWritePin(AnalogPin.P13, motorspeedline)
                pins.digitalWritePin(DigitalPin.P14, 1)
                pins.digitalWritePin(DigitalPin.P15, 1)
                pins.analogWritePin(AnalogPin.P16, motorspeedline)

            }
            while (Port > Ref) {

            }
            pins.digitalWritePin(DigitalPin.P13, 1)
            pins.digitalWritePin(DigitalPin.P14, 1)
            pins.digitalWritePin(DigitalPin.P15, 1)
            pins.digitalWritePin(DigitalPin.P16, 1)
            basic.pause(100)
        }
        if (rotateLINE == turn.Right) {
            pins.analogWritePin(AnalogPin.P13, motorspeedline)
            pins.digitalWritePin(DigitalPin.P14, 1)
            pins.analogWritePin(AnalogPin.P16, motorspeedline)
            pins.digitalWritePin(DigitalPin.P15, 1)
            while (Port < Ref) {
                pins.digitalWritePin(DigitalPin.P13, 1)
                pins.analogWritePin(AnalogPin.P14, motorspeedline)
                pins.analogWritePin(AnalogPin.P15, motorspeedline)
                pins.digitalWritePin(DigitalPin.P16, 1)
            }
            while (Port > Ref) {

            }
            pins.digitalWritePin(DigitalPin.P13, 1)
            pins.digitalWritePin(DigitalPin.P14, 1)
            pins.digitalWritePin(DigitalPin.P15, 1)
            pins.digitalWritePin(DigitalPin.P16, 1)
            basic.pause(100)
        }
    }

    /**
     * Execute puase time
     * @param pausetime  	mSec to delay; eg: 100
    */
    /**Motor Block to drives motor forward and backward. The speed motor is adjustable between 0 to 100.
      * @param speed percent of maximum speed, eg: 50
      */
    //% blockId="iKB1_Motor" block="iKB1 Motor %iKB1Motor|speed %speed"
    //% speed.min=0 speed.max=100
    //% weight=37
    export function Motor(Motor: iKB1Motor, speed: number): void {
        if (Motor == iKB1Motor.Forward) {
            pins.i2cWriteNumber(72, (0x23 * 256) + speed, NumberFormat.UInt16BE, false)
        }
        else if (Motor == iKB1Motor.Backward) {
            pins.i2cWriteNumber(72, (0x23 * 256) + (256 - speed), NumberFormat.UInt16BE, false)
        }
    }

    //% blockId="IKB_reset" block="iKB Reset"
    //% weight=36
    export function Reset(): void {
        pins.i2cWriteNumber(72, 0, NumberFormat.UInt8BE, false)
    }



    /**MotorCH set Motor Channel and Direction. The speed motor is adjustable between 0 to 100.   
     * @param Speed percent of maximum Speed, eg: 50
     */
    //% blockId="iKB1_MotorCH" block="setMotor %iKB1MotorCH | Direction %iKB1Motor | Speed %Speed"
    //% Speed.min=0 Speed.max=100
    //% weight=35
    export function setMotor(Channel: iKB1MotorCH, Direction: iKB1Motor, Speed: number): void {
        if (Direction == iKB1Motor.Forward) {
            pins.i2cWriteNumber(72, (Channel * 256) + (Speed), NumberFormat.UInt16BE, false)
        }
        else if (Direction == iKB1Motor.Backward) {
            pins.i2cWriteNumber(72, (Channel * 256) + (256 - Speed), NumberFormat.UInt16BE, false)
        }

    }


    /**Spin Block set direction SpinLeft or SpinRight. The speed motor is adjustable between 0 to 100.  
      * @param speed percent of maximum speed, eg: 50
      */
    //% blockId="iKB1_Spin" block="iKB1 Spin %iKB1Spin|speed %speed"
    //% speed.min=0 speed.max=100
    //% weight=34
    export function Spin(Spin: iKB1Spin, speed: number): void {
        if (Spin == iKB1Spin.Left) {
            pins.i2cWriteNumber(72, (0x21 * 256) + (speed), NumberFormat.UInt16BE, false)
            pins.i2cWriteNumber(72, (0x22 * 256) + (256 - speed), NumberFormat.UInt16BE, false)
        }
        else if (Spin == iKB1Spin.Right) {
            pins.i2cWriteNumber(72, (0x22 * 256) + (speed), NumberFormat.UInt16BE, false)
            pins.i2cWriteNumber(72, (0x21 * 256) + (256 - speed), NumberFormat.UInt16BE, false)
        }
    }

    /**Turn Block set direction TurnLeft or TurnRight. The speed motor is adjustable between 0 to 100.
      * @param speed percent of maximum speed, eg: 50
      */
    //% blockId="iKB1_Turn" block="iKB1 Turn %iKB1Turn|speed %speed"
    //% speed.min=0 speed.max=100
    //% weight=33
    export function Turn(Turn: iKB1Turn, speed: number): void {
        if (Turn == iKB1Turn.Left) {
            pins.i2cWriteNumber(72, (0x22 * 256) + (speed), NumberFormat.UInt16BE, false)
            pins.i2cWriteNumber(72, (0x21 * 256) + (0), NumberFormat.UInt16BE, false)
        }
        else if (Turn == iKB1Turn.Right) {
            pins.i2cWriteNumber(72, (0x21 * 256) + (speed), NumberFormat.UInt16BE, false)
            pins.i2cWriteNumber(72, (0x22 * 256) + (0), NumberFormat.UInt16BE, false)
        }
    }

    //% blockId="AO" block="Motor Stop"
    //% weight=32
    export function AO(): void {
        pins.i2cWriteNumber(72, (0x23 * 256) + (0), NumberFormat.UInt16BE, false)
    }
}
