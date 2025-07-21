
// @config( microbit.boardVersion == 2 )

/**
 * braillebot blocks
 */
//% weight=100 color=#3333FF icon="\uf0fe"
namespace braillebot {

    // 핀 설정

    const redPin = DigitalPin.P13
    const greenPin = DigitalPin.P14
    const bluePin = DigitalPin.P15

    const servoPin = AnalogPin.P16

    const VEML6040_ADDR = 0x10

    const BLACK_THRESHOLD = 500

    let tracking = false
    let allConnected = false
    let melodyMode = false

    let baseSpeed = 70
    let deviation = 20
    let move_deviation = 3
    let blindColor = 0

    // PID 변수
    let setpoint = 0
    let pid_input = 0
    let pid_output = 0
    let prevInput = 0
    let integral = 0
    let Kp = 3.5
    let Ki = 0.0
    let Kd = 0.2


    let colorCount = 0
    let lineExist = 0


    let leftValue = 0
    let rightValue = 0
    let balanceCount = 0
    let leftBalance = 0
    let rightBalance = 0

    let leftOld = 0
    let rightOld = 0

    let colorkey = 0
    let objectGap = 0

    let red = 0
    let green = 0
    let blue = 0
    let redBalance = 0
    let greenBalance = 0
    let blueBalance = 0
    let normred = 0
    let normgreen = 0
    let normblue = 0
    let rg = 0
    let rb = 0
    let gr = 0
    let gb = 0
    let br = 0
    let bg = 0

    const RED_KEY = 1 // AUTO
    const GREEN_KEY = 2
    const BLUE_KEY = 3 // AUTO
    const CYAN_KEY = 4
    const MAGENTA_KEY = 5 // AUTO
    const YELLOW_KEY = 6
    const ORANGE_KEY = 7 // AUTO
    const PINK_KEY = 8
    const BLACK_KEY = 9
    const WHITE_KEY = 10

    const GCUBE_SET_LINEBOARD_SENSOR_DATA = 0x12 // ~ 2024-11-22
    const GCUBE_REQ_LINEBOARD_SPEED = 0x13 // ~ 2024-11-22
    const GCUBE_REQ_LINEBOARD_MOVE = 0x14 // ~ 2024-11-22
    const GCUBE_REQ_LINEBOARD_ROTATE = 0x15 // ~ 2024-11-22
    const GCUBE_REQ_LINEBOARD_MOVETOCOLOR = 0x16 // ~ 2024-11-22
    const GCUBE_REQ_LINEBOARD_ROTATETOLINE = 0x17 // ~ 2024-11-22
    const GCUBE_REQ_LINEBOARD_GRIPPER = 0x18 // ~ 2024-11-22
    const GCUBE_REQ_LINEBOARD_MOVETOGRID = 0x19 // ~ 2024-11-26
    const GCUBE_REQ_LINEBOARD_MELODY = 0x1B // ~ 2025-05-29
    const GCUBE_REQ_LINEBOARD_REBOOT = 0x1E // ~ 2025-05-05
    const GCUBE_CONTROL_COMMAND = 0x1F // ~ 2024-12-14
    const GCUBE_GET_BOARD_ID = 0x10 // 보드의 ID 값 확인 요청
    const GCUBE_LINE_BOARD_ID = 0x72 // 자동차형 모델 큐브 2개에 꽂혀 동작하는 트레이싱 전용 보드 ID : 0x72 ~ 2024-10-08

    export enum Note {
        Do = 262,
        Re = 294,
        Mi = 330,
        Fa = 349,
        Sol = 392,
        La = 440,
        Si = 494,
        HighDo = 523
    }

    export enum Checking {
        None = 0,
        Skip = 1
    }

    serial.redirect(SerialPin.P2, SerialPin.P1, 115200)
    serial.setRxBufferSize(10)
    serial.setTxBufferSize(10)


    function veml6040_init() {

        let buf = pins.createBuffer(3)
        buf[0] = 0x00 //CMD register
        buf[1] = 0x10
        buf[2] = 0x00
        pins.i2cWriteBuffer(VEML6040_ADDR, buf)

        basic.pause(200)
    }

    function readColorRegister(register: number): number {

        pins.i2cWriteNumber(VEML6040_ADDR, register, NumberFormat.UInt8BE)
        basic.pause(10)
        return pins.i2cReadNumber(VEML6040_ADDR, NumberFormat.UInt16LE)
    }

    function readRed(): number {
        return readColorRegister(0x08)
    }

    function readGreen(): number {
        return readColorRegister(0x09)
    }

    function readBlue(): number {
        return readColorRegister(0x0A)
    }

    // ... (이전 코드 유지)

    function colorName(color: number): string {
        switch (color) {
            case RED_KEY: return "RED"
            case GREEN_KEY: return "GREEN"
            case BLUE_KEY: return "BLUE"
            case CYAN_KEY: return "CYAN"
            case MAGENTA_KEY: return "MAGENTA"
            case YELLOW_KEY: return "YELLOW"
            case ORANGE_KEY: return "ORANGE"
            case PINK_KEY: return "PINK"
            case BLACK_KEY: return "BLACK"
            case WHITE_KEY: return "WHITE"
            default: return ""
        }
    }


    function showColor(color: number) {
        switch (color) {
            case RED_KEY:
                pins.digitalWritePin(redPin, 0)
                pins.digitalWritePin(greenPin, 1)
                pins.digitalWritePin(bluePin, 1)
                break
            case YELLOW_KEY:
                pins.digitalWritePin(redPin, 0)
                pins.digitalWritePin(greenPin, 0)
                pins.digitalWritePin(bluePin, 1)
                break
            case GREEN_KEY:
                pins.digitalWritePin(redPin, 1)
                pins.digitalWritePin(greenPin, 0)
                pins.digitalWritePin(bluePin, 1)
                break
            case BLUE_KEY:
                pins.digitalWritePin(redPin, 1)
                pins.digitalWritePin(greenPin, 1)
                pins.digitalWritePin(bluePin, 0)
                break
            case ORANGE_KEY:
                pins.digitalWritePin(redPin, 0)
                pins.digitalWritePin(greenPin, 0)
                pins.digitalWritePin(bluePin, 0)
                break
            case MAGENTA_KEY:
                pins.digitalWritePin(redPin, 0)
                pins.digitalWritePin(greenPin, 1)
                pins.digitalWritePin(bluePin, 0)
                break
            case CYAN_KEY:
                pins.digitalWritePin(redPin, 1)
                pins.digitalWritePin(greenPin, 0)
                pins.digitalWritePin(bluePin, 0)
                break
            case PINK_KEY:
                pins.digitalWritePin(redPin, 1)
                pins.digitalWritePin(greenPin, 1)
                pins.digitalWritePin(bluePin, 1)
                break
            case BLACK_KEY:
                pins.digitalWritePin(redPin, 1)
                pins.digitalWritePin(greenPin, 1)
                pins.digitalWritePin(bluePin, 1)
                break
            default:
                pins.digitalWritePin(redPin, 1)
                pins.digitalWritePin(greenPin, 1)
                pins.digitalWritePin(bluePin, 1)
                break
        }
    }

    function meetColor(): boolean {
        let returnKey = false

        // VEML6040에서 RGB 값 읽기
        red = readRed()
        green = readGreen()
        blue = readBlue()

        // 검정색이면 밸런스 카운트 증가
        if (red + green + blue < 1300) {
            balanceCount += 1
        } else {
            balanceCount = 0
        }

        // 기준값 기반 정규화
        normred = red / redBalance
        normgreen = green / greenBalance
        normblue = blue / blueBalance

        rg = normred / normgreen
        rb = normred / normblue
        gr = normgreen / normred
        gb = normgreen / normblue
        br = normblue / normred
        bg = normblue / normgreen

        // 색상이 존재하는지 확인
        if (Math.abs(rg - gr) + Math.abs(rb - br) + Math.abs(gb - bg) > 0.9) {
            returnKey = true
        }

        // 특정 조건 하에서 라인트래킹 종료
        if (tracking && blindColor == 0) {
            let tempColor = 0

            if (
                normred < 0.3 && normgreen < 0.3 && normblue < 0.3 &&
                (Math.abs(normred - normblue) + Math.abs(normgreen - normblue) + Math.abs(normred - normgreen)) < 0.1 &&
                gr < rg
                || red + green + blue < 1000
            ) {
                tempColor = 1
            }

            if (tempColor == 0) {
                lineExist++
            } else {
                lineExist = 0
            }

            if (lineExist > 10) {
                tracking = false
                basic.pause(20)
                motorSpeedControl(0, 0)
                lineExist = 0
                showColor(0)
            }
        } else {
            lineExist = 0
        }

        return returnKey
    }




    function detectColorKey(): number {
        let returnKey = 0

        // VEML6040에서 RGB 읽기
        red = readRed()
        green = readGreen()
        blue = readBlue()

        normred = red / redBalance
        normgreen = green / greenBalance
        normblue = blue / blueBalance

        rg = normred / normgreen
        rb = normred / normblue
        gr = normgreen / normred
        gb = normgreen / normblue
        br = normblue / normred
        bg = normblue / normgreen

        if (rg > 1.2) {
            let At = rg + gr
            let Bt = rb + gb
            let Ct = rg + bg
            let Et = br + bg
            let It = rg + rb + bg

            if (At + 0.1 < Bt) {
                if (At + 0.15 < Ct) {
                    if (rg >= 1.45) {
                        returnKey = RED_KEY
                    } else {
                        if (gb > 1.1) {
                            returnKey = ORANGE_KEY
                        } else {
                            returnKey = RED_KEY
                        }
                    }
                } else if (rb > 1.5) {
                    if (It > 4.8) {
                        returnKey = RED_KEY
                    } else {
                        returnKey = ORANGE_KEY
                    }
                }
            } else {
                if (At < Et) {
                    if (normred > 0.5) {
                        returnKey = PINK_KEY
                    } else {
                        returnKey = MAGENTA_KEY
                    }
                } else {
                    returnKey = PINK_KEY
                }
            }

            if (br > 2.0 || bg > 2.0) {
                returnKey = BLUE_KEY
            }
        } else if (gb > 1.2) {
            let Ct = rg + bg
            let Dt = gr + br
            let Ft = rg + rb
            let Gt = gr + gb
            let Ht = Math.abs(Ft - Gt) + Math.abs(Ct - Dt)

            if (Ht < 0.25) {
                if (gb > 1.4) returnKey = YELLOW_KEY
            } else {
                if (Ft < Gt) {
                    returnKey = GREEN_KEY
                } else {
                    if (Gt < Ct) {
                        if (rg > 1.35) returnKey = RED_KEY
                    } else if (gb > 1.5) {
                        if (Ht > 0.8) returnKey = ORANGE_KEY
                        else returnKey = YELLOW_KEY
                    }
                }
            }
        } else {
            if (normblue > 0.5 && normred > 0.4 && normgreen > 0.4 && br > 1.2) {
                returnKey = CYAN_KEY
            } else if (br > 1.2 && bg > 1.2 && normred < 0.4) {
                returnKey = BLUE_KEY
            }
        }

        if (
            (normred < 0.3 && normgreen < 0.3 && normblue < 0.3 &&
                (Math.abs(normred - normblue) + Math.abs(normgreen - normblue) + Math.abs(normred - normgreen)) < 0.12 &&
                gr < rg) ||
            (red + green + blue) < 1000
        ) {
            returnKey = BLACK_KEY
        }

        if (normred > 0.8 && normgreen > 0.8 && normblue > 0.8) {
            returnKey = WHITE_KEY
        }

        if (red + green + blue < 700) {
            returnKey = 0
        }

        return returnKey
    }



    function doBalance(mode: number) {
        let rv = 0
        let gv = 0
        let bv = 0
        let ls = 0
        let rs = 0
        let whitecheck = true

        if (mode == 1) {
            colorkey = detectColorKey()
            if (colorkey != YELLOW_KEY) return
            else {
                // 노란색 LED 점멸 5초
                pins.digitalWritePin(redPin, 1)
                pins.digitalWritePin(greenPin, 1)
                pins.digitalWritePin(bluePin, 1)
                for (let a = 0; a < 10; a++) {
                    if (a % 2 == 0) {
                        pins.digitalWritePin(redPin, 0)
                        pins.digitalWritePin(greenPin, 0)
                    } else {
                        pins.digitalWritePin(redPin, 1)
                        pins.digitalWritePin(greenPin, 1)
                    }
                    basic.pause(500)
                }
                showColor(BLUE_KEY)
            }
        }

        for (let i = 0; i < 5; i++) {
            basic.pause(130)
            colorkey = detectColorKey()

            if ((red + green + blue) < 4000) {
                whitecheck = false
            }

            let ll = pins.analogReadPin(AnalogPin.P3)
            let rr = pins.analogReadPin(AnalogPin.P4)
            rv += red
            gv += green
            bv += blue
            ls += ll
            rs += rr
        }

        rv /= 5
        gv /= 5
        bv /= 5
        ls /= 5
        rs /= 5

        if (whitecheck) {
            // EEPROM 대신 settings 사용
            //settings.writeNumber("redBalance", rv)
            //settings.writeNumber("greenBalance", gv)
            //settings.writeNumber("blueBalance", bv)
            //settings.writeNumber("leftBalance", ls)
            //settings.writeNumber("rightBalance", rs)

            redBalance = rv
            greenBalance = gv
            blueBalance = bv
            leftBalance = ls
            rightBalance = rs

            showColor(0) // LED OFF
        } else {
            // showColor(BLUE_KEY) 유지
        }
    }



    function checkWhiteBalance(mode: number) {
        if (mode == 0) {
            // 초기 셋업 시 저장된 값 불러오기
            //let rv = settings.readNumber("redBalance")
            //let gv = settings.readNumber("greenBalance")
            //let bv = settings.readNumber("blueBalance")
            //let ls = settings.readNumber("leftBalance")
            //let rs = settings.readNumber("rightBalance")

            let rv = 0;
            let gv = 0;
            let bv = 0;
            let ls = 0;
            let rs = 0;

            if (rv == 0 || gv == 0 || bv == 0) {
                // white balance 수동 요청
                pins.digitalWritePin(redPin, 1) // RED on
                pins.digitalWritePin(greenPin, 1) // GREEN on
                pins.digitalWritePin(bluePin, 1) // BLUE on
                for (let a = 0; a < 10; a++) {
                    if (a % 2 == 0) pins.digitalWritePin(redPin, 0) // RED on
                    else pins.digitalWritePin(redPin, 1) // RED off
                    basic.pause(500)
                }
                showColor(BLUE_KEY) // BLUE_KEY 대체
                doBalance(0)
            } else {
                redBalance = rv
                greenBalance = gv
                blueBalance = bv
                leftBalance = ls
                rightBalance = rs
            }
        } else {
            // loop 중 자동 재보정 조건
            if (balanceCount > 102 && tracking == false) {
                let leftValue = pins.analogReadPin(AnalogPin.P3)
                let rightValue = pins.analogReadPin(AnalogPin.P4)
                leftValue = Math.map(leftValue, leftBalance, 1023, 0, 1023)
                rightValue = Math.map(rightValue, rightBalance, 1023, 0, 1023)
                if (leftValue < 0) leftValue = 0
                if (rightValue < 0) rightValue = 0

                if ((leftValue > 300 && leftValue < 900) && (rightValue > 300 && rightValue < 900)) {
                    pins.digitalWritePin(redPin, 1) // RED on
                    pins.digitalWritePin(greenPin, 1) // GREEN
                    pins.digitalWritePin(bluePin, 1) // BLUE
                    for (let a = 0; a < 10; a++) {
                        if (a % 2 == 0) pins.digitalWritePin(redPin, 0) // RED
                        else pins.digitalWritePin(redPin, 1)
                        basic.pause(500)
                    }
                    showColor(BLUE_KEY)
                    doBalance(1)
                }
                balanceCount = 0
            }
        }
    }


    function computePID() {
        let error = setpoint - pid_input
        integral += error
        let derivative = pid_input - prevInput
        pid_output = Kp * error + Ki * integral - Kd * derivative
        pid_output = Math.constrain(pid_output, -1 * deviation, deviation)
        prevInput = pid_input
    }

    function direct_send_gcube(p: number[], serialPort: String) {
        const buffer = pins.createBufferFromArray(p)

        if (serialPort == "left") {
            serial.redirect(SerialPin.P2, SerialPin.P1, 115200)
        } else {
            serial.redirect(SerialPin.P12, SerialPin.P8, 115200)
        }

        serial.writeBuffer(buffer)
        basic.pause(1)
    }

    function get_iv(cmd: number): number {
        return Math.floor(cmd / 16) + (cmd & 0x0F) * 16
    }

    function direct_cube_speed_control(motor_speed: number, colorkey: number, distance: number, isRightCube: boolean) {
        if (motor_speed < -100) motor_speed = -100
        else if (motor_speed > 100) motor_speed = 100

        let buffer = pins.createBuffer(10)
        buffer.setUint8(0, GCUBE_REQ_LINEBOARD_SPEED)
        buffer.setUint8(1, get_iv(GCUBE_REQ_LINEBOARD_SPEED))
        buffer.setUint8(2, 0)
        buffer.setUint8(3, motor_speed)  // signed
        buffer.setUint8(4, colorkey)
        buffer.setUint8(5, distance)
        buffer.setUint8(6, 0)
        buffer.setUint8(7, 0)
        buffer.setUint8(8, 0)
        buffer.setUint8(9, 0)

        if (isRightCube) {
            serial.redirect(SerialPin.P12, SerialPin.P8, BaudRate.BaudRate115200)
        } else {
            serial.redirect(SerialPin.P2, SerialPin.P1, BaudRate.BaudRate115200)
        }

        serial.writeBuffer(buffer)
    }


    function direct_cube_move_control(motor_speed: number, robot_distance: number, serialPort: String) {
        if (robot_distance < 0) {
            motor_speed = motor_speed * -1
        }
        robot_distance = Math.abs(robot_distance)
        if (robot_distance > 200) {
            robot_distance = 200
        }
        direct_send_gcube([GCUBE_REQ_LINEBOARD_MOVE, get_iv(GCUBE_REQ_LINEBOARD_MOVE), 0, motor_speed, robot_distance, 0, 0, 0, 0, 0], serialPort)
    }

    function direct_cube_rotate_control(motor_speed: number, robot_angle: number, serialPort: String) {
        if (robot_angle < 0) {
            motor_speed = motor_speed * -1
        }
        robot_angle = Math.abs(robot_angle)
        if (robot_angle > 180) {
            robot_angle = 180
        }
        direct_send_gcube([GCUBE_REQ_LINEBOARD_ROTATE, get_iv(GCUBE_REQ_LINEBOARD_ROTATE), 0, motor_speed, robot_angle, 0, 0, 0, 0, 0], serialPort)
    }

    function direct_cube_melody_control(melody_number: number, m1: number, m2: number, m3: number, serialPort: String) {
        direct_send_gcube([GCUBE_REQ_LINEBOARD_MELODY, get_iv(GCUBE_REQ_LINEBOARD_ROTATE), melody_number, m1, m2, m3, 0, 0, 0, 0], serialPort)
    }



    function motorSpeedControl(leftSpeed: number, rightSpeed: number) {
        if (Math.abs(leftOld - leftSpeed) > 1) {
            direct_cube_speed_control(-1 * leftSpeed, colorkey, objectGap, false)
            leftOld = leftSpeed
        }
        if (Math.abs(rightOld - rightSpeed) > 1) {
            direct_cube_speed_control(rightSpeed, colorkey, objectGap, true)
            rightOld = rightSpeed
        }
    }

    function moveRobot(speed: number, distance: number) {
        direct_cube_move_control(speed, distance, "right") // G2
        direct_cube_move_control(-1 * speed, distance, "left") // G1

        if (Math.abs(speed) >= 10) {
            let delayTime = Math.idiv(10000 * Math.abs(distance), Math.abs(speed))
            basic.pause(delayTime + 100)
        }
    }

    function rotateRobot(speed: number, angle: number) {
        direct_cube_rotate_control(-1 * speed, angle, "right") // G2
        direct_cube_rotate_control(-1 * speed, angle, "left") // G1

        if (Math.abs(speed) >= 10) {
            let delayTime = Math.idiv(667 * Math.abs(angle), Math.abs(speed))
            basic.pause(delayTime + 50)
        }
    }


    function wait_for_lineboard_cube_connected(mode: number) {
        let rcvData: number[] = [0, 0, 0]
        let cubeNumber = 0

        // P2 RX (Cube 1), P12 RX (Cube 2)
        pins.setPull(DigitalPin.P2, PinPullMode.PullUp)
        let pinState = pins.digitalReadPin(DigitalPin.P2)
        if (pinState == 1) {
            direct_send_gcube([GCUBE_CONTROL_COMMAND, get_iv(GCUBE_CONTROL_COMMAND), 1, 0, 0, 0, 0, 0, 0, 0], "left")
        }
        pins.setPull(DigitalPin.P12, PinPullMode.PullUp)
        pinState = pins.digitalReadPin(DigitalPin.P12)
        if (pinState == 1) {
            direct_send_gcube([GCUBE_CONTROL_COMMAND, get_iv(GCUBE_CONTROL_COMMAND), 1, 0, 0, 0, 0, 0, 0, 0], "right")
        }

        serial.redirect(SerialPin.P2, SerialPin.P1, 115200)
        let buf = serial.readBuffer(3)
        for (let i = 0; i < 3; i++) {
            rcvData[i] = buf.getUint8(i)  // 각 바이트를 배열에 복사
        }

        if (rcvData[0] == GCUBE_GET_BOARD_ID && rcvData[1] == 0x00 && rcvData[2] == 0x00) {
            direct_send_gcube([GCUBE_GET_BOARD_ID, get_iv(GCUBE_GET_BOARD_ID), 0, 0, 0, GCUBE_LINE_BOARD_ID, 0, 0, 0, 0], "left")
            cubeNumber++
        }

        serial.redirect(SerialPin.P12, SerialPin.P8, 115200)
        buf = serial.readBuffer(3)
        for (let i = 0; i < 3; i++) {
            rcvData[i] = buf.getUint8(i)  // 각 바이트를 배열에 복사
        }

        if (rcvData[0] == GCUBE_GET_BOARD_ID && rcvData[1] == 0x00 && rcvData[2] == 0x00) {
            direct_send_gcube([GCUBE_GET_BOARD_ID, get_iv(GCUBE_GET_BOARD_ID), 0, 0, 0, GCUBE_LINE_BOARD_ID, 0, 0, 0, 0], "right")
            cubeNumber++
        }

        if (cubeNumber >= mode) return
    }


    function rotateUntilDetectLine(direction: number): boolean {
        let count = 0

        if (direction == 0) {
            motorSpeedControl(-35, 35) // turn left
        } else {
            motorSpeedControl(35, -35) // turn right
        }

        basic.pause(30)

        for (count = 0; count < 40; count++) {
            colorkey = detectColorKey()

            if (colorkey == BLACK_KEY) {
                motorSpeedControl(0, 0)
                break
            } else if (colorkey == ORANGE_KEY || colorkey == CYAN_KEY) {
                motorSpeedControl(0, 0)
                showColor(colorkey)
                break
            } else if (colorkey == GREEN_KEY) {
                motorSpeedControl(0, 0)
                showColor(GREEN_KEY)
                break
            }

            // IR 센서 라인 감지
            let lineValue = 0
            if (direction == 1) {
                lineValue = pins.map(pins.analogReadPin(AnalogPin.P3), leftBalance, 1023, 0, 1023)
            } else {
                lineValue = pins.map(pins.analogReadPin(AnalogPin.P4), rightBalance, 1023, 0, 1023)
            }

            if (lineValue > 500) {
                motorSpeedControl(0, 0)
                break
            }

            basic.pause(80)
        }

        basic.pause(200)

        if (count == 40) {
            motorSpeedControl(0, 0)
            return false
        } else {
            showColor(0)
            return true
        }
    }

    function checkGrid(): boolean {
        return (leftValue > BLACK_THRESHOLD && rightValue > BLACK_THRESHOLD)
    }


    function mapToRange(val: number, fromLow: number, fromHigh: number, toLow: number, toHigh: number): number {
        return Math.idiv((val - fromLow) * (toHigh - toLow), (fromHigh - fromLow)) + toLow
    }


    /**
     * Setup braille bot
     */
    //% block="Setup braille bot"
    export function setupBrailleBot(): void {
        pins.digitalWritePin(redPin, 1) // RED Off
        pins.digitalWritePin(greenPin, 1) // GREEN Off
        pins.digitalWritePin(bluePin, 1) // BLUE Off

//        pins.servoWritePin(servoPin, 90)

        pins.digitalWritePin(DigitalPin.P7, 0) // System LED ON

        basic.pause(500)


        veml6040_init()

        checkWhiteBalance(0)

        wait_for_lineboard_cube_connected(2)

        allConnected = true

        pins.digitalWritePin(DigitalPin.P7, 0) // System LED OFF


    }


    //% block="Read Color key"
    export function getColorKey(): number {
        let colorNumber = detectColorKey();
        return colorNumber
    }


    //% block="Show Color with $colorNumber"
    export function showColorKey(colorNumber: number): void {
        if (0 < colorNumber && colorNumber < 9) showColor(colorNumber);
    }


    /**
     * 두 개의 음을 1박자씩 연주하기
     * @param note1 1st tone
     * @param note2 2nd tone
     */
    //% block="PlayTone 1st: %note1| 2nd: %note2"
    export function playTwoNotes(note1: Note, note2: Note): void {
        music.playTone(note1, music.beat(BeatFraction.Quarter))
        music.playTone(note2, music.beat(BeatFraction.Quarter))
        basic.pause(1000)
    }


    //% block="Set Echo ON"
    export function setEchoOn(): void {
        melodyMode = true
    }


    //% block="Turn Left"
    export function turnLeft(): void {
        moveRobot(baseSpeed, move_deviation)
        rotateRobot(-1 * baseSpeed, 55)
        let detection_flag = rotateUntilDetectLine(0)
    }


    //% block="Turn Right"
    export function turnRight(): void {
        moveRobot(baseSpeed, move_deviation)
        rotateRobot(baseSpeed, 55)
        let detection_flag = rotateUntilDetectLine(1)
    }


    //% block="U turn"
    export function uTurn(): void {
        moveRobot(baseSpeed, 2)
        rotateRobot(-50, 90)
        let detection_flag = rotateUntilDetectLine(0)
    }


    //% block="Line tracking to next color %mode"
    export function lineTrackingToNextColor(mode: Checking): void {

        colorCount = 0;

        while (true) {

            if (tracking) {
                leftValue = pins.analogReadPin(AnalogPin.P3) // Left IR
                rightValue = pins.analogReadPin(AnalogPin.P4) // Right IR

                leftValue = mapToRange(leftValue, leftBalance, 1023, 0, 1023)
                rightValue = mapToRange(rightValue, rightBalance, 1023, 0, 1023)

                if (leftValue < 0) leftValue = 0
                if (rightValue < 0) rightValue = 0

                pid_input = (-1 * leftValue + rightValue) / 64.0
                computePID() // PID calculation

                motorSpeedControl(baseSpeed - pid_output, baseSpeed + pid_output)
            }

            colorCount++

            basic.pause(16) // 16ms line tracking

            if (colorCount % 6 == 0 && mode == 0) { // every 16*6 = 96 msec check the Color sensor
                let existColor = meetColor()
                if (existColor) {
                    basic.pause(60) //wait for reading position
                    break
                }
            }

            if (mode == 1 && colorCount > 42) break  // exit skip near color mode ORANGE/CYAN
            if (colorCount > 625) break  // Searching next color time limit 10 msec
        }
        motorSpeedControl(0, 0)
        basic.pause(50)
    }


    //% block="Gripper Open $mode"
    export function gripperOpenBlock(mode: boolean): void {
        if (mode) {
            let detection_flag = false
            motorSpeedControl(0, 0)
            basic.pause(300)

            moveRobot(baseSpeed, 9)
            basic.pause(300)

            pins.servoWritePin(servoPin, 35)
            basic.pause(300)

            moveRobot(-1 * baseSpeed, 8)
            basic.pause(300)

            pins.servoWritePin(servoPin, 90)
            basic.pause(300)

            rotateRobot(-50, 60)
            detection_flag = rotateUntilDetectLine(0)

            if (detection_flag) {
                tracking = true
            }
        } else {
            basic.pause(300)
            pins.servoWritePin(servoPin, 35)
            basic.pause(300)
        }
    }


    //% block="Gripper Close $mode"
    export function gripperCloseBlock(mode: boolean): void {
        if (mode) {
            let detection_flag = false
            motorSpeedControl(0, 0)
            pins.servoWritePin(servoPin, 35)
            basic.pause(300)

            moveRobot(baseSpeed, 9)
            basic.pause(300)

            for (let i = 40; i <= 90; i += 5) {
                pins.servoWritePin(servoPin, i)
                basic.pause(100)
            }

            basic.pause(300)
            moveRobot(-1 * baseSpeed, 8)
            basic.pause(300)

            rotateRobot(-50, 60)
            detection_flag = rotateUntilDetectLine(0)

            if (detection_flag) {
                tracking = true
            }

        } else {
            basic.pause(300)
            for (let i = 40; i <= 90; i += 5) {
                pins.servoWritePin(servoPin, i)
                basic.pause(100)
            }
            basic.pause(300)
        }
    }


    //% block="Move forward $distance cm , with speed : $speed %"
    export function moveBraillebot(speed: number, distance: number): void {
        moveRobot(speed, distance)
    }


    //% block="Rotate $degree degree , with speed : $speed %"
    export function rotateBraillebot(speed: number, degree: number): void {
        rotateRobot(speed, degree)
    }


    //% block="Set motor speed Left: $left % and Right: $right"
    export function setMotorSpeed(left: number, right: number): void {
        motorSpeedControl(left, right)
    }

    //% block="Show data"
    export function showData(): void{
        //let tempRed = pins.analogReadPin(AnalogPin.P4)
        let tempData = detectColorKey()

        tempData = normblue
        basic.showNumber(tempData, 100)
    }


}