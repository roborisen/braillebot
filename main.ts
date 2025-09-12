
// @config( microbit.boardVersion == 2 )

/**
 * braillebot blocks
 */
//% weight=100 color=#3333FF icon="\uf0fe"
namespace braillebot {

    // 핀 설정


    const redPin = DigitalPin.P13
    const greenPin = DigitalPin.P9
    const bluePin = DigitalPin.P10

    const servoPin = AnalogPin.P16

    const VEML6040_ADDR = 0x10

    const BLACK_THRESHOLD = 500

    let allConnected = false
    let melodyMode = false
    let melodyAction = true

    let baseSpeed = 60
    let speedDeviation = 30
    let moveDeviation = 4
    let blindColor = 0

    // PID 변수
    let setPoint = 0
    let pidInput = 0
    let pidOutput = 0
    let prevInput = 0
    let integralSum = 0
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

    let colorKey = 0
    let objectGap = 0
    let oldColor = 0

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
    const VIOLET_KEY = 5 // AUTO
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
        //% block="C4"
        C4 = 262,
        //% block="D4"
        D4 = 294,
        //% block="E4"
        E4 = 330,
        //% block="F4"
        F4 = 349,
        //% block="G4"
        G4 = 392,
        //% block="A4"
        A4 = 440,
        //% block="B4"
        B4 = 494,
        //% block="C5"
        C5 = 523
    }

    export enum Action {
        //% block="Action"
        Action = 0,
        //% block="Stop"
        Stop = 1
    }

    export enum Icons {
        //% block="Move forward"
        MoveForward = 0,
        //% block="Turn left"
        TurnLeft = 1,
        //% block="Turn right"
        TurnRight = 2,
        //% block="U turn"
        UTurn = 3,
        //% block="Stop"
        Stop = 4,
        //% block="Gripper open"
        GripperOpen = 5,
        //% block="Gripper close"
        GripperClose = 6
    }

    // Definition of Colorkey
    export enum inKey {
        //% block="RED"
        RED_KEY = 1,
        //% block="GREEN"
        GREEN_KEY = 2,
        //% block="BLUE"
        BLUE_KEY = 3,
        //% block="CYAN"
        CYAN_KEY = 4,
        //% block="VIOLET"
        VIOLET_KEY = 5,
        //% block="YELLOW"
        YELLOW_KEY = 6,
        //% block="ORANGE"
        ORANGE_KEY = 7,
        //% block="PINK"
        PINK_KEY = 8
    }

    export enum Opening {
        //% block="Just open"
        JustOpen = 0,
        //% block="Moving open"
        MovingOpen = 1
    }

    export enum Closing {
        //% block="Just close"
        JustClose = 0,
        //% block="Moving close"
        MovingClose = 1
    }

    export enum Checking {
        //% block="Without skip"
        None = 0,
        //% block="Skip the near color"
        Skip = 1
    }


    function veml6040Begin(): boolean {
        let sensorExists = false
        try {
            pins.i2cWriteNumber(VEML6040_ADDR, 0x00, NumberFormat.UInt8BE)
            sensorExists = true
        } catch (e) {
            sensorExists = false
        }
        return sensorExists
    }


    function veml6040Init() {

        let buf = pins.createBuffer(3)
        buf[0] = 0x00 //CMD register
        buf[1] = 0x10
        buf[2] = 0x00
        pins.i2cWriteBuffer(VEML6040_ADDR, buf)
    }

    function readColorRegister(register: number): number {

        pins.i2cWriteNumber(VEML6040_ADDR, register, NumberFormat.UInt8BE, true)
        basic.pause(2)

        let buf = pins.i2cReadBuffer(VEML6040_ADDR, 2)
        let low = buf.getUint8(0)
        let high = buf.getUint8(1)

        return (high << 8) | low  // Little Endian (VEML6040 규격)
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

    function showColor(color: number) {
        led.enable(false)
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
            case VIOLET_KEY:
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
//        if (Math.abs(rg - gr) + Math.abs(rb - br) + Math.abs(gb - bg) > 0.9) {// 0.9->1.0 2025-09-10
        if (Math.abs(rg - gr) + Math.abs(rb - br) + Math.abs(gb - bg) > 1.2) {// 0.9->1.0 2025-09-10
            returnKey = true
        }

        // 특정 조건 하에서 라인트래킹 종료
        if (blindColor == 0) {
            let tempColor = 0

            if (
                normred < 0.3 && normgreen < 0.3 && normblue < 0.3 &&
                (Math.abs(normred - normblue) + Math.abs(normgreen - normblue) + Math.abs(normred - normgreen)) < 0.1 &&
                gr < rg
//                || red + green + blue < 1000
                || red + green + blue < 2000  //2025-08-27
            ) {
                tempColor = 1
            }

            if (tempColor == 0) {
                lineExist++
            } else {
                lineExist = 0
            }

            if (lineExist > 15) {
                motorSpeedControl(0, 0)
                lineExist = 0
                showColor(RED_KEY)
            }
        } else {
            lineExist = 0
        }

        return returnKey
    }



    // ---- RGB -> HSV (캘리브레이션/정규화 포함) ----
    function rgbToHsv(rRaw: number, gRaw: number, bRaw: number): { h: number, s: number, v: number } {
        // 1) 캘리브레이션 및 정규화
        const r = rRaw / redBalance
        const g = gRaw / greenBalance
        const b = bRaw / blueBalance

        // 0~1 스케일(필요 시 최대값으로 나눌 수 있음; 여기선 비율만 사용)
        const cMax = Math.max(r, Math.max(g, b))
        const cMin = Math.min(r, Math.min(g, b))
        const diff = cMax - cMin

        // V
        const v = cMax
        if (cMax === 0) {
            return { h: 0, s: 0, v: 0 }
        }

        // S
        const s = diff / cMax

        // H
        let hDeg = 0
        if (diff === 0) {
            hDeg = 0
        } else if (cMax === r) {
            // ((g - b) / diff) % 6, 음수 처리
            const t = (g - b) / diff
            const mod6 = ((t % 6) + 6) % 6
            hDeg = 60 * mod6
        } else if (cMax === g) {
            hDeg = 60 * (((b - r) / diff) + 2)
        } else { // cMax === b
            hDeg = 60 * (((r - g) / diff) + 4)
        }

        if (hDeg < 0) hDeg += 360
        return { h: hDeg, s: s, v: v }
    }

    function detectColorKey():number {

        red = readRed()
        green = readGreen()
        blue = readBlue()

        const hsv = rgbToHsv(red, green, blue)
        const h = hsv.h
        const s = hsv.s
        const v = hsv.v

        // 검정/흰색 우선 판별
        if (s > 0.05 && s < 0.20 && v < 0.35) {
            return BLACK_KEY
        }
        if (s < 0.1 && v > 0.9) {
            return WHITE_KEY
        }

        // 유색 판별
        if (s > 0.2) {
            if (h >= 350 || h < 15) return RED_KEY      // 빨강
            if (h >= 19 && h < 35) return ORANGE_KEY   // 주황
//            if (h >= 50 && h < 65) return YELLOW_KEY   // 노랑
//            if (h >= 65 && h < 150) return GREEN_KEY    // 초록
            if (h >= 50 && h < 68) return YELLOW_KEY   // 노랑 //2025-09-10
            if (h >= 68 && h < 150) return GREEN_KEY    // 초록 //2025-09-10
            if (h >= 180 && h < 245 && Math.abs(s-v) > 0.2) return CYAN_KEY    // 청록, 2025-09-10
            if (h >= 230 && h < 260) return BLUE_KEY    // 파랑
            if (h >= 260 && h < 325) return VIOLET_KEY // 마젠타
            if (h >= 325 && h < 350) return PINK_KEY    // 핑크
        }

        return 0
    }


    function doBalance():boolean {
        let rv = 0
        let gv = 0
        let bv = 0
        let ls = 0
        let rs = 0
        let whitecheck = true

        for (let i = 0; i < 5; i++) {
            basic.pause(130)
            colorKey = detectColorKey()

            if ((red + green + blue) < 3500) { //2025-09-12
                whitecheck = false
            }

            let ll = pins.analogReadPin(AnalogPin.P1)
            let rr = pins.analogReadPin(AnalogPin.P2)
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
            return true
        } else {
            return false
        }
    }



    function checkWhiteBalance():boolean {
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
            for (let a = 0; a < 4; a++) {
                if (a % 2 == 0) pins.digitalWritePin(redPin, 0) // RED on
                else pins.digitalWritePin(redPin, 1) // RED off
                basic.pause(500)
            }
            showColor(BLUE_KEY) // BLUE_KEY 대체
            return doBalance()
        } else {
            redBalance = rv
            greenBalance = gv
            blueBalance = bv
            leftBalance = ls
            rightBalance = rs
            return true
        }

    }


    function computePID() {
        let error = setPoint - pidInput
        integralSum += error
        let derivative = pidInput - prevInput
        pidOutput = Kp * error + Ki * integralSum - Kd * derivative
        pidOutput = Math.constrain(pidOutput, -1 * speedDeviation, speedDeviation)
        prevInput = pidInput
    }

    function directSendGcube(p: number[], serialPort: String) {
        const buffer = pins.createBufferFromArray(p)

        if (serialPort == "left") {
            serial.redirect(SerialPin.P14, SerialPin.P15, 115200)
        } else {
            serial.redirect(SerialPin.P12, SerialPin.P8, 115200)
        }

        basic.pause(10) //2025-08-30
        serial.writeBuffer(buffer)
        basic.pause(30)
    }

    function getInvert(cmd: number): number {
        return Math.floor(cmd / 16) + (cmd & 0x0F) * 16
    }

    function directCubeSpeedControl(motorSpeed: number, colorkey: number, distance: number, isRightCube: boolean) {
        if (motorSpeed < -100) motorSpeed = -100
        else if (motorSpeed > 100) motorSpeed = 100

        let buffer = pins.createBuffer(10)
        buffer.setUint8(0, GCUBE_REQ_LINEBOARD_SPEED)
        buffer.setUint8(1, getInvert(GCUBE_REQ_LINEBOARD_SPEED))
        buffer.setUint8(2, 0)
        buffer.setUint8(3, motorSpeed)  // signed
        buffer.setUint8(4, colorkey)
        buffer.setUint8(5, distance)
        buffer.setUint8(6, 0)
        buffer.setUint8(7, 0)
        buffer.setUint8(8, 0)
        buffer.setUint8(9, 0)

        if (isRightCube) {
            serial.redirect(SerialPin.P12, SerialPin.P8, BaudRate.BaudRate115200)
        } else {
            serial.redirect(SerialPin.P14, SerialPin.P15, BaudRate.BaudRate115200)
        }

        serial.writeBuffer(buffer)
    }


    function directCubeMoveControl(motorSpeed: number, robotDistance: number, serialPort: String) {
        if (robotDistance < 0) {
            motorSpeed = motorSpeed * -1
        }
        robotDistance = Math.abs(robotDistance)
        if (robotDistance > 200) {
            robotDistance = 200
        }
        directSendGcube([GCUBE_REQ_LINEBOARD_MOVE, getInvert(GCUBE_REQ_LINEBOARD_MOVE), 0, motorSpeed, robotDistance, 0, 0, 0, 0, 0], serialPort)
    }

    function directCubeRotateControl(motorSpeed: number, robotAngle: number, serialPort: String) {
        if (robotAngle < 0) {
            motorSpeed = motorSpeed * -1
        }
        robotAngle = Math.abs(robotAngle)
        if (robotAngle > 180) {
            robotAngle = 180
        }

        directSendGcube([GCUBE_REQ_LINEBOARD_ROTATE, getInvert(GCUBE_REQ_LINEBOARD_ROTATE), 0, motorSpeed, robotAngle, 0, 0, 0, 0, 0], serialPort)
    }

    function directCubeMelodyControl(melodyNumber: number, m1: number, m2: number, m3: number, serialPort: String) {
        directSendGcube([GCUBE_REQ_LINEBOARD_MELODY, getInvert(GCUBE_REQ_LINEBOARD_MELODY), melodyNumber, m1, m2, m3, 0, 0, 0, 0], serialPort)
    }



    function motorSpeedControl(leftSpeed: number, rightSpeed: number) {
        if (Math.abs(leftOld - leftSpeed) > 1) {
            directCubeSpeedControl(-1 * leftSpeed, colorKey, objectGap, false)
            leftOld = leftSpeed
        }
        if (Math.abs(rightOld - rightSpeed) > 1) {
            directCubeSpeedControl(rightSpeed, colorKey, objectGap, true)
            rightOld = rightSpeed
        }
    }

    function moveRobot(speed: number, distance: number) {
        directCubeMoveControl(speed, distance, "right") // G2
        directCubeMoveControl(-1 * speed, distance, "left") // G1

        if (Math.abs(speed) >= 10) {
            let delayTime = Math.idiv(10000 * Math.abs(distance), Math.abs(speed))
            basic.pause(delayTime + 100)
        }
    }

    function rotateRobot(speed: number, angle: number) {
        directCubeRotateControl(-1 * speed, angle, "right") // G2
        directCubeRotateControl(-1 * speed, angle, "left") // G1

        if (Math.abs(speed) >= 10) {
            let delayTime = Math.idiv(667 * Math.abs(angle), Math.abs(speed))
            basic.pause(delayTime + 50)
        }
    }


    function waitForLineboardCubeConnected(mode: number) {
        let rcvData: number[] = [0, 0, 0]
        let cube1Connected = false
        let cube2Connected = false
        let timeout = 0

        while (!(cube1Connected && cube2Connected)) {
            // 큐브1 (P14: RX, P15: TX)
            if (!cube1Connected) {
                pins.setPull(DigitalPin.P14, PinPullMode.PullUp)
                let pinState1 = pins.digitalReadPin(DigitalPin.P14)
                if (pinState1 == 1) {
                    directSendGcube([GCUBE_CONTROL_COMMAND, getInvert(GCUBE_CONTROL_COMMAND), 1, 0, 0, 0, 0, 0, 0, 0], "left")
                    serial.redirect(SerialPin.P14, SerialPin.P15, 115200)
                    basic.pause(50)
                    let buf1 = serial.readBuffer(3)
                    if (buf1.length == 3) {
                        for (let i = 0; i < 3; i++) {
                            rcvData[i] = buf1.getUint8(i)
                        }
                        if (rcvData[0] == GCUBE_GET_BOARD_ID && rcvData[1] == 0x00 && rcvData[2] == 0x00) {
                            directSendGcube([GCUBE_GET_BOARD_ID, getInvert(GCUBE_GET_BOARD_ID), 0, 0, 0, GCUBE_LINE_BOARD_ID, 0, 0, 0, 0], "left")
                            cube1Connected = true
                        }
                    }
                }
            }

            basic.pause(50) // 너무 빠른 루프 방지

            // 큐브2 (P12: RX, P8: TX)
            if (!cube2Connected) {
                pins.setPull(DigitalPin.P12, PinPullMode.PullUp)
                let pinState2 = pins.digitalReadPin(DigitalPin.P12)
                if (pinState2 == 1) {
                    directSendGcube([GCUBE_CONTROL_COMMAND, getInvert(GCUBE_CONTROL_COMMAND), 1, 0, 0, 0, 0, 0, 0, 0], "right")
                    serial.redirect(SerialPin.P12, SerialPin.P8, 115200)
                    basic.pause(50)
                    let buf2 = serial.readBuffer(3)
                    if (buf2.length == 3) {
                        for (let i = 0; i < 3; i++) {
                            rcvData[i] = buf2.getUint8(i)
                        }
                        if (rcvData[0] == GCUBE_GET_BOARD_ID && rcvData[1] == 0x00 && rcvData[2] == 0x00) {
                            directSendGcube([GCUBE_GET_BOARD_ID, getInvert(GCUBE_GET_BOARD_ID), 0, 0, 0, GCUBE_LINE_BOARD_ID, 0, 0, 0, 0], "right")
                            cube2Connected = true
                        }
                    }
                }
            }

            basic.pause(50) // 너무 빠른 루프 방지

            timeout++
            if (timeout > 200) {// 예: 20초 후에도 연결 안 되면 실패
                break
            }
        }
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
            colorKey = detectColorKey()

            if (colorKey == BLACK_KEY) {
                motorSpeedControl(0, 0)
                break
            } 
            //else if (colorKey == GREEN_KEY) {
            //    motorSpeedControl(0, 0)
            //    showColor(GREEN_KEY)
            //    break
            //}

            // IR 센서 라인 감지
            let lineValue = 0
            if (direction == 1) {
                lineValue = pins.map(pins.analogReadPin(AnalogPin.P1), leftBalance, 1023, 0, 1023)
            } else {
                lineValue = pins.map(pins.analogReadPin(AnalogPin.P2), rightBalance, 1023, 0, 1023)
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
//            showColor(0) //2025-08-30
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
     * Rotate the robot with a speed
     * @param angle the angle of rotation
     * @param speed robot speed
     */
    //% block="Rotate $degree degree with speed : $speed %"
    export function rotateBraillebot(speed: number, degree: number): void {
        rotateRobot(speed, degree)
    }


    /**
     * Move the robot with a speed
     * @param distance robot distance
     * @param speed robot speed
     */
    //% block="Move forward $distance cm with speed : $speed %"
    export function moveBraillebot(speed: number, distance: number): void {
        moveRobot(speed, distance)
    }

    /**
     * Set the speed of each wheel of the robot
     * @param left speed of left wheel of the robot
     * @param right speed of right wheel of the robot
     */
    //% block="Set motor speed Left: $left % and Right: $right %"
    export function setMotorSpeed(left: number, right: number): void {
        motorSpeedControl(left, right)
    }


    /**
     * Show the icon with the corresponding motion of the robot
     * @param icon Display the icon for the corresponding robot motion
     */
    //% block="showIcon %icon"
    export function showIcon(icon: Icons): void {
        /*
            MoveForward = 0,
            TurnLeft = 1,
            TurnRight = 2,
            UTurn = 3,
            Stop = 4,
            GripperOpen = 5,
            GripperClose = 6
        */
        led.enable(true)

        if (icon == 0) {
            basic.clearScreen()
            led.plot(2, 0)
            led.plot(1, 1)
            led.plot(2, 1)
            led.plot(3, 1)
            led.plot(2, 2)
            led.plot(2, 3)
            led.plot(2, 4)
        } else if (icon == 1) {
            basic.clearScreen()
            led.plot(0, 2)
            led.plot(1, 1)
            led.plot(1, 2)
            led.plot(1, 3)
            led.plot(2, 2)
            led.plot(3, 2)
            led.plot(4, 2)
        } else if (icon == 2) {
            basic.clearScreen()
            led.plot(4, 2)
            led.plot(3, 1)
            led.plot(3, 2)
            led.plot(3, 3)
            led.plot(2, 2)
            led.plot(1, 2)
            led.plot(0, 2)
        } else if (icon == 3) {
            basic.clearScreen()
            led.plot(2, 4)
            led.plot(1, 3)
            led.plot(2, 3)
            led.plot(3, 3)
            led.plot(2, 2)
            led.plot(2, 1)
            led.plot(2, 0)

        } else if (icon == 4) {
            basic.clearScreen()
            led.plot(2, 2)

        } else if (icon == 5) {
            basic.clearScreen()
            led.plot(0, 1)
            led.plot(0, 2)
            led.plot(0, 3)
            led.plot(1, 0)
            led.plot(1, 4)
            led.plot(2, 0)
            led.plot(2, 4)
            led.plot(3, 0)
            led.plot(3, 4)
            led.plot(4, 1)
            led.plot(4, 2)
            led.plot(4, 3)
        } else if (icon == 6) {
            basic.clearScreen()
            led.plot(2, 1)
            led.plot(2, 2)
            led.plot(2, 3)
            led.plot(1, 4)
            led.plot(2, 4)
            led.plot(3, 4)
        }

    }


    /**
     * Play two notes for one beat each
     * @param note1 1st tone
     * @param note2 2nd tone
     * @param mode Melody for Action status or Stop status
     */
    //% block="Play two tones 1st: %note1| 2nd: %note2  Mode: %mode"
    export function playTwoNotes(note1: Note, note2: Note, mode: Action): void {

        if (!melodyMode) return

        if (mode === 0) {
            music.playTone(note1, music.beat(BeatFraction.Quarter))
            basic.pause(50)
            music.playTone(note2, music.beat(BeatFraction.Quarter))
            basic.pause(500)
        } else {
            if (melodyAction) { // one time play when the robot is staying in stop position
                melodyAction = false
                oldColor = colorKey
                music.playTone(note1, music.beat(BeatFraction.Quarter))
                basic.pause(50)
                music.playTone(note2, music.beat(BeatFraction.Quarter))
                basic.pause(500)
            }
        }
    }



    /**
     * Set melody mode on
     */
    //% block="Set Echo ON"
    export function setEchoOn(): void {
        melodyMode = true
    }


    /**
     * Close gripper
     * @param mode Simple (Closing only) or Full (Moving + Closing)
     */
    //% block="Gripper Close $mode"
    export function gripperCloseBlock(mode: Closing): void {
        if (mode) {
            let detectionFlag = false
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
            detectionFlag = rotateUntilDetectLine(0)

        } else {
            basic.pause(300)
            for (let i = 40; i <= 90; i += 5) {
                pins.servoWritePin(servoPin, i)
                basic.pause(100)
            }
            basic.pause(300)
        }
    }


    /**
     * Open gripper
     * @param mode Simple (Opening only) or Full (Moving + Opening)
     */
    //% block="Gripper Open $mode"
    export function gripperOpenBlock(mode: Opening): void {
        if (mode) {
            let detectionFlag = false
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
            detectionFlag = rotateUntilDetectLine(0)

        } else {
            basic.pause(300)
            pins.servoWritePin(servoPin, 35)
            basic.pause(300)
        }
    }



    /**
     * Skip adjace color and start line tracking to the next color detection
     */
    //% block="Line tracking while skipping adjacent colors"
    export function lineTrackingSkipAndNextColor(): void {

        colorCount = 0

        let mode = 1

        while (true) {

            leftValue = pins.analogReadPin(AnalogPin.P1) // Left IR
            rightValue = pins.analogReadPin(AnalogPin.P2) // Right IR

            leftValue = mapToRange(leftValue, leftBalance, 1023, 0, 1023)
            rightValue = mapToRange(rightValue, rightBalance, 1023, 0, 1023)

            if (leftValue < 0) leftValue = 0
            if (rightValue < 0) rightValue = 0

            pidInput = (-1 * leftValue + rightValue) / 64.0
            computePID() // PID calculation

            motorSpeedControl(baseSpeed - pidOutput, baseSpeed + pidOutput)

            colorCount++

            basic.pause(16) // 16ms line tracking

            if (colorCount % 6 == 0 && mode == 0) { // every 16*6 = 96 msec check the Color sensor
                let existColor = meetColor()
                if (existColor) {
                    basic.pause(30) //wait for reading position

                    //2025-09-10
                    let tempColor = detectColorKey()
                    if (tempColor > 0 && tempColor < 9) break
                }
            }

            if (mode == 1 && colorCount > 50) mode = 0  // change mode after moving a distance
            if (colorCount > 625) break  // Searching next color time limit 10 msec
        }
        motorSpeedControl(0, 0)
        basic.pause(50)
    }


    /**
     * Line tracking to the next color detection
     */
    //% block="Line tracking to the next color"
    export function lineTrackingToNextColor(): void {

        colorCount = 0;

        while (true) {

            leftValue = pins.analogReadPin(AnalogPin.P1) // Left IR
            rightValue = pins.analogReadPin(AnalogPin.P2) // Right IR

            leftValue = mapToRange(leftValue, leftBalance, 1023, 0, 1023)
            rightValue = mapToRange(rightValue, rightBalance, 1023, 0, 1023)

            if (leftValue < 0) leftValue = 0
            if (rightValue < 0) rightValue = 0

            pidInput = (-1 * leftValue + rightValue) / 64.0
            computePID() // PID calculation

            motorSpeedControl(baseSpeed - pidOutput, baseSpeed + pidOutput)

            colorCount++

            basic.pause(16) // 16ms line tracking

            // 30*16 = 480 msec after start
            if (colorCount % 6 == 0 && colorCount > 15) { // every 16*6 = 96 msec check the Color sensor
                let existColor = meetColor()
                if (existColor) {
                    basic.pause(30) //wait for reading position

                    //2025-09-10
                    let tempColor = detectColorKey()
                    if (tempColor > 0 && tempColor < 9) break
               }
            }

            if (colorCount > 625) break  // Searching next color time limit 10 msec
        }
        motorSpeedControl(0, 0)
        basic.pause(50)
    }


    /**
     * Stop the robot
     */
    //% block="Stop"
    export function stop(): void {
        motorSpeedControl(0, 0)
    }


    /**
     * Take U turn
     */
    //% block="U turn"
    export function uTurn(): void {
        moveRobot(baseSpeed, 2)
        rotateRobot(-50, 120)
        let detectionFlag = rotateUntilDetectLine(0)
    }


    /**
     * Turn right
     */
    //% block="Turn Right"
    export function turnRight(): void {
        moveRobot(baseSpeed, moveDeviation)
        rotateRobot(baseSpeed, 55)
        let detectionFlag = rotateUntilDetectLine(1)
    }


    /**
     * Turn left
     */
    //% block="Turn Left"
    export function turnLeft(): void {
        moveRobot(baseSpeed, moveDeviation)
        rotateRobot(-1 * baseSpeed, 55)
        let detectionFlag = rotateUntilDetectLine(0)
    }


    /**
     * Turn on LED based on detected color
     * @param mode Simple (Opening only) or Full (Moving + Opening)
     */
    //% block="Display LED color with $colorNumber"
    export function showColorKey(colorNumber: number): void {
        if (0 < colorNumber && colorNumber < 9) showColor(colorNumber);
        if (oldColor != colorNumber) melodyAction = true
    }

    /**
     * When a color is detected
     */
    //% block="If %color color is detected"
    export function isColorDetected(color: inKey): boolean {
        if (colorKey == color) return true
        else return false
    }


    /**
     * Read color sensor
     */
    //% block="Read Color Sensor"
    export function readColorSensor(): void {
        colorKey = detectColorKey();
        if( colorKey > 0 && colorKey <9){ //2025-08-30
            basic.pause(200)
            colorKey = detectColorKey(); //Check color again
        }
    }


    /**
     * Get detected color key
     */
    //% block="Colorkey"
    export function getColorKey(): number {
        return colorKey
    }


    /**
     * Initialize Braille bot
     */
    //% block="Initialize Braille bot"
    export function setupBrailleBot(): void {
        pins.digitalWritePin(redPin, 1) // RED Off
        pins.digitalWritePin(greenPin, 1) // GREEN Off
        pins.digitalWritePin(bluePin, 1) // BLUE Off

        pins.servoWritePin(servoPin, 90)

        pins.digitalWritePin(DigitalPin.P7, 1) // System LED ON

        serial.redirect(SerialPin.P14, SerialPin.P15, 115200)
        serial.setRxBufferSize(10)
        serial.setTxBufferSize(10)

        veml6040Init()

        basic.pause(500)

        if(checkWhiteBalance()){
            music.play(music.tonePlayable(262, music.beat(BeatFraction.Quarter)), music.PlaybackMode.UntilDone)
        }else{
            music.play(music.tonePlayable(523, music.beat(BeatFraction.Whole)), music.PlaybackMode.UntilDone)
        }

        pins.digitalWritePin(DigitalPin.P7, 0) // System LED OFF

        waitForLineboardCubeConnected(2)

        allConnected = true

        basic.pause (500)

    }


}