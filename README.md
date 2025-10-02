# BrailleBot MakeCode Extension

This is the official MakeCode extension for the **BrailleBot** robot, designed to work with micro\:bit and G-Cube-based tracing and manipulation robots.

**BrailleBot** is a robot developed to provide coding and robotics education for students with visual impairments. By integrating tactile feedback and line-tracing functions, it enables inclusive learning experiences.

This extension provides blocks for movement, color detection, PID line tracking, gripper control, melody playback, and more.

The information of BrailleBot is available in the following link

<BraillBot user guide>(https://drive.google.com/drive/u/0/folders/1hTqSkhzJzLy-aDYCrle0bNnitDoQOqee)

And the BrailleBot is available in

<BraillBot product link>(https://global.gmarket.co.kr/item?goodscode=4549651616)


## 🧩 Blocks Overview

### 🚗 Movement Blocks

* **Move forward `$distance` cm with speed: `$speed` %**
  Move the robot forward by a given distance and speed.

* **Rotate `$degree` degree with speed: `$speed` %**
  Rotates the robot by a specified angle and speed.

* **Set motor speed Left: `$left` % and Right: `$right` %**
  Directly control each motor's speed.

* **Stop**
  Immediately stops both motors.

* **Turn Left / Turn Right / U turn**
  Predefined turning blocks with alignment to the next line.

### 🎨 Color Blocks

* **Read Color Sensor**
  Reads color values from the VEML6040 sensor.

* **Colorkey**
  Returns the currently detected color key (mapped internally).

* **Show Color with `$colorNumber`**
  Displays an LED color corresponding to the detected color.

### 📈 Line Tracking Blocks

* **Line tracking to next color**
  Follows the line until the robot detects the next color.

* **Line tracking while skipping adjacent colors**
  Ignores nearby colors briefly and then begins tracking to the next color.

### 🦾 Gripper Control Blocks

* **Gripper Close `$mode`**
  Closes the servo-based gripper. Mode can be simple or full (move + close).

* **Gripper Open `$mode`**
  Opens the gripper. Mode can be simple or full (move + open).

### 🎵 Melody and Icon Blocks

* **Play tones 1st: `$note1`, 2nd: `$note2`, Mode: `$mode`**
  Plays two tones depending on robot action or stop status.

* **Set Echo ON**
  Enables melody mode.

* **showIcon `$icon`**
  Displays predefined motion-related icons (e.g., forward, stop, U-turn, etc.)

### ⚙️ Setup Block

* **Setup BrailleBot**
  Initializes pins, sensor, white balance, and robot connection.

## 🧪 Color Codes

Internally, the robot uses the following codes for color identification:

* `1`: RED
* `2`: GREEN
* `3`: BLUE
* `4`: CYAN
* `5`: MAGENTA
* `6`: YELLOW
* `7`: ORANGE
* `8`: PINK
* `9`: BLACK
* `10`: WHITE

## 🛠️ Dependencies

* micro\:bit v2
* G-Cube communication over UART
* VEML6040 I2C Color Sensor
* *G-Cube Serial #1 RX : P14 (Microbit TX)	
* *G-Cube Serial #1 TX : P15 (Microbit RX)
* *G-Cube Serial #2 RX : P12 (Microbit TX)
* *G-Cube Serial #2 TX : P8 (Microbit RX)
* *IR sensor Left :	P1
* *IR sensor Right :	P2
* *Indication LED :	P7
* *Color LED - R :	P13
* *Color LED - G :	P9
* *Color LED - B :	P10
* *RC servoe :		P16
* *VEML6040 SDA :	P20
* *VEML6040 SCL :	P19
  

## 📦 Installation

To add this extension in MakeCode:

1. Go to the **Advanced** section in the MakeCode editor
2. Click **Extensions**
3. Search for `braillebot` or paste the GitHub repository URL


# BrailleBot MakeCode Extension

This extension provides blocks to control the BrailleBot robot with line tracing, gripper control, color detection, and melody features using the micro:bit.

## API Reference

### `setupBrailleBot()`
**Block:** Initialize BrailleBot  
Initializes all robot systems, including color sensor, servos, communication ports, and calibration.

---

### `moveBraillebot(speed: number, distance: number)`
**Block:** Move forward `$distance` cm with speed: `$speed` %  
Moves the robot forward or backward a given distance at the specified speed.

---

### `rotateBraillebot(speed: number, degree: number)`
**Block:** Rotate `$degree` degree with speed: `$speed` %  
Rotates the robot left or right by a specified degree at the given speed.

---

### `setMotorSpeed(left: number, right: number)`
**Block:** Set motor speed Left: `$left` % and Right: `$right` %  
Sets the speed of the left and right motors directly.

---

### `stop()`
**Block:** Stop  
Stops all motor movement immediately.

---

### `uTurn()`
**Block:** U turn  
Executes a U-turn maneuver using forward and rotational movements.

---

### `turnRight()`
**Block:** Turn Right  
Turns the robot right by a predefined angle.

---

### `turnLeft()`
**Block:** Turn Left  
Turns the robot left by a predefined angle.

---

### `lineTrackingToNextColor()`
**Block:** Line tracking to the next color  
Follows the line until a new color is detected using the color sensor.

---

### `lineTrackingSkipAndNextColor()`
**Block:** Line tracking while skipping adjacent colors  
Skips immediately detected color and continues line tracing until the next color.

---

### `readColorSensor()`
**Block:** Read Color Sensor  
Reads the current color using the VEML6040 sensor and stores the result.

---

### `getColorKey(): number`
**Block:** Colorkey  
Returns the currently detected color key (e.g., RED_KEY, GREEN_KEY, etc.).

---

### `showColorKey(colorNumber: number)`
**Block:** Display LED color with `$colorNumber`  
Shows an RGB LED color on the robot based on the provided color key number.

---

### `gripperOpenBlock(mode: Opening)`
**Block:** Gripper Open `$mode`  
Opens the gripper. If mode is `MovingOpen`, the robot moves forward and then opens the gripper.

---

### `gripperCloseBlock(mode: Closing)`
**Block:** Gripper Close `$mode`  
Closes the gripper. If mode is `MovingClose`, the robot moves forward before closing the gripper.

---

### `playTwoNotes(note1: Note, note2: Note, mode: Action)`
**Block:** Play two tones 1st: `$note1`, 2nd: `$note2`, Mode: `$mode`  
Plays two musical notes. In `Stop` mode, the melody plays only once when robot is idle.

---

### `setEchoOn()`
**Block:** Set Echo ON  
Enables melody playing mode for `playTwoNotes`.

---

### `showIcon(icon: Icons)`
**Block:** showIcon `$icon`  
Displays a motion icon (e.g., forward, turn, stop) on the LED matrix.

---

## Enums

- `Note`: Musical notes (C4, D4, E4, etc.)
- `Action`: Playback trigger mode (`Action`, `Stop`)
- `Icons`: Motion symbols to display on LED
- `Opening`: Gripper open mode (`JustOpen`, `MovingOpen`)
- `Closing`: Gripper close mode (`JustClose`, `MovingClose`)
- `Checking`: Color skipping mode (`None`, `Skip`)

---

## Examples
### Braille mission example code
```blocks
braillebot.setupBrailleBot()
music.play(music.builtinPlayableSoundEffect(soundExpression.hello), music.PlaybackMode.UntilDone)
basic.forever(function () {
    braillebot.readColorSensor()
    braillebot.showColorKey(braillebot.readColorKey())
    if (braillebot.isColorDetected(braillebot.InKey.RedKey)) {
        braillebot.playTwoNotes(braillebot.Note.G4, braillebot.Note.B4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.TurnRight)
        braillebot.turnRight()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.BlueKey)) {
        braillebot.playTwoNotes(braillebot.Note.G4, braillebot.Note.E4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.TurnLeft)
        braillebot.turnLeft()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.YellowKey)) {
        braillebot.playTwoNotes(braillebot.Note.G4, braillebot.Note.G4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.UTurn)
        braillebot.uTurn()
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.OrangeKey)) {
        basic.pause(500)
        braillebot.showIcon(braillebot.Icons.MoveForward)
        braillebot.lineTrackingSkipAndNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.CyanKey)) {
        braillebot.setEchoOn()
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.MoveForward)
        braillebot.lineTrackingSkipAndNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.PinkKey)) {
        braillebot.playTwoNotes(braillebot.Note.C4, braillebot.Note.C5, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.GripperOpen)
        braillebot.gripperOpenBlock(braillebot.Opening.MovingOpen)
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.VioletKey)) {
        braillebot.playTwoNotes(braillebot.Note.C5, braillebot.Note.C4, braillebot.Action.Action)
        braillebot.showIcon(braillebot.Icons.GripperClose)
        braillebot.gripperCloseBlock(braillebot.Closing.MovingClose)
        braillebot.lineTrackingToNextColor()
    } else if (braillebot.isColorDetected(braillebot.InKey.GreenKey)) {
        braillebot.playTwoNotes(braillebot.Note.C5, braillebot.Note.C5, braillebot.Action.Stop)
        braillebot.showIcon(braillebot.Icons.Stop)
    } else {
    	
    }
})

```
<a href="https://makecode.microbit.org/S72593-51524-53977-14891">This example</a> shows <br/>
1. wait until two G-Cubes are connected to microbit <br/>
2. read the color of the braille coin and make an action along the color<br/>
3. run the mission untile the robot meet the stop(end) command color<br/>
<br/>

## 🧑‍💻 License

MIT License

---

Created by Robo Risen
For support or inquiries, contact: qna@roborisen.com
