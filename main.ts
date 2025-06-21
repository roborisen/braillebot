
// @config( microbit.boardVersion == 2 )

/**
 * braillebot blocks
 */
//% weight=100 color=#111111 icon="\uf0fe"
namespace braillebot {
     
    // 핀 설정
    const leftIR = AnalogPin.P3     // 예시: IR 센서 왼쪽
    const rightIR = AnalogPin.P4    // 예시: IR 센서 오른쪽
    const servoPin = AnalogPin.P16 //RC Servo motor

    const redPin = DigitalPin.P13
    const greenPin = DigitalPin.P14
    const bluePin = DigitalPin.P15


    const VEML6040_ADDR = 0x10

    const BLACK_THRESHOLD = 500

    let tracking = false
    let autoMode = false
    let reading = false
    let allConnected = true
    let MelodyMode = false

    let baseSpeed = 70
    let deviation = 20
    let move_deviation = 3
    let blindColor = 0

    // PID 변수
    let setpoint = 0
    let pin_input = 0
    let pin_output = 0
    let Kp = 3.5
    let Ki = 0.0
    let Kd = 0.2


    let colorCount = 0
    let sonicCount = 3
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
    let At = 0
    let Bt = 0
    let Ct = 0
    let Dt = 0
    let Et = 0
    let Ft = 0
    let Gt = 0
    let Ht = 0
    let It = 0

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


    serial.redirect(SerialPin.P1, SerialPin.P2, 115200)
    serial.setRxBufferSize(10)
    serial.setTxBufferSize(10)



    function veml6040_init() {
        const config = pins.createBuffer(3)
        config.setUint8(0, 0x00)           // 레지스터 주소 0x00
        config.setUint8(1, 0x09)           // 레지스터 주소 0x01
        config.setUint8(2, 0x00)           // 레지스터 주소 0x02
        pins.i2cWriteBuffer(VEML6040_ADDR, config)
        basic.pause(200)
    }

    function veml6040_checkConnection(): boolean {
        // 간단히 색상 레지스터 하나 읽어서 0이 아니면 정상으로 판단
        pins.i2cWriteNumber(VEML6040_ADDR, 0x08, NumberFormat.UInt8BE)
        let r = pins.i2cReadNumber(VEML6040_ADDR, NumberFormat.UInt16LE)
        return r > 0
    }

    function readColorRegister(register: number): number {
        pins.i2cWriteNumber(VEML6040_ADDR, register, NumberFormat.UInt8BE)
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
        reading = true
        red = readRed()
        green = readGreen()
        blue = readBlue()
        reading = false

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
        reading = true
        red = readRed()
        green = readGreen()
        blue = readBlue()
        reading = false

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

            let ll = pins.analogReadPin(leftIR)
            let rr = pins.analogReadPin(rightIR)
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
            settings.writeNumber("redBalance", rv)
            settings.writeNumber("greenBalance", gv)
            settings.writeNumber("blueBalance", bv)
            settings.writeNumber("leftBalance", ls)
            settings.writeNumber("rightBalance", rs)

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
            let rv = settings.readNumber("redBalance")
            let gv = settings.readNumber("greenBalance")
            let bv = settings.readNumber("blueBalance")
            let ls = settings.readNumber("leftBalance")
            let rs = settings.readNumber("rightBalance")

            if (rv == 0 || gv == 0 || bv == 0) {
                // white balance 수동 요청
                pins.digitalWritePin(DigitalPin.P13, 1) // RED on
                pins.digitalWritePin(DigitalPin.P14, 1) // GREEN on
                pins.digitalWritePin(DigitalPin.P15, 1) // BLUE on
                for (let a = 0; a < 10; a++) {
                    if (a % 2 == 0) pins.digitalWritePin(DigitalPin.P13, 0) // RED on
                    else pins.digitalWritePin(DigitalPin.P13, 1) // RED off
                    basic.pause(500)
                }
                showColor(3) // BLUE_KEY 대체
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
                    pins.digitalWritePin(DigitalPin.P13, 1) // RED on
                    pins.digitalWritePin(DigitalPin.P14, 1) // GREEN
                    pins.digitalWritePin(DigitalPin.P15, 1) // BLUE
                    for (let a = 0; a < 10; a++) {
                        if (a % 2 == 0) pins.digitalWritePin(DigitalPin.P13, 0) // RED
                        else pins.digitalWritePin(DigitalPin.P13, 1)
                        basic.pause(500)
                    }
                    showColor(3)
                    doBalance(1)
                }
                balanceCount = 0
            }
        }
    }










    /**
     * start braille bot
     */
    //% block="Initialize braille bot"
    export function startBrailleBot(): void {
        pins.digitalWritePin(DigitalPin.P13, 1) // RED Off
        pins.digitalWritePin(DigitalPin.P14, 1) // GREEN Off
        pins.digitalWritePin(DigitalPin.P15, 1) // BLUE Off

        pins.servoWritePin(AnalogPin.P16, 90)

        pins.digitalWritePin(DigitalPin.P7, 1) // System LED ON

        basic.pause(500)

        veml6040_init()

    }









}