/**
 * braillebot blocks
 */
//% weight=100 color=#111111 icon="\uf0fe"
namespace braillebot {
    
    // 핀 설정
    const leftIR = AnalogPin.P0     // 예시: IR 센서 왼쪽
    const rightIR = AnalogPin.P1    // 예시: IR 센서 오른쪽
    const modePin = DigitalPin.P5
    const trigPin = DigitalPin.P6
    const echoPin = DigitalPin.P7
    const redPin = DigitalPin.P8
    const greenPin = DigitalPin.P9
    const bluePin = DigitalPin.P10
    const servoPin = DigitalPin.P11
    const powerPin = DigitalPin.P12

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


    serial.redirect(SerialPin.P1, SerialPin.P2, 115200)
    serial.setRxBufferSize(10)
    serial.setTxBufferSize(10)

    /**
     * start braille bot
     */
    //% block="Initialize braille bot"
    export function startBrailleBot(): void {
        
    }









}