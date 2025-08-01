# BrailleBot MakeCode Extension

This is the official MakeCode extension for the **BrailleBot** robot, designed to work with micro\:bit and G-Cube-based tracing and manipulation robots.

**BrailleBot** is a robot developed to provide coding and robotics education for students with visual impairments. By integrating tactile feedback and line-tracing functions, it enables inclusive learning experiences.

This extension provides blocks for movement, color detection, PID line tracking, gripper control, melody playback, and more.

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

* **Setup braille bot**
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

## 📦 Installation

To add this extension in MakeCode:

1. Go to the **Advanced** section in the MakeCode editor
2. Click **Extensions**
3. Search for `braillebot` or paste the GitHub repository URL


# BrailleBot MakeCode Extension

This extension provides blocks to control the BrailleBot robot with line tracing, gripper control, color detection, and melody features using the micro:bit.

## API Reference

### `setupBrailleBot()`
**Block:** Initialize Braille bot  
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

- `Note`: Musical notes (Do, Re, Mi, etc.)
- `Action`: Playback trigger mode (`Action`, `Stop`)
- `Icons`: Motion symbols to display on LED
- `Opening`: Gripper open mode (`JustOpen`, `MovingOpen`)
- `Closing`: Gripper close mode (`JustClose`, `MovingClose`)
- `Checking`: Color skipping mode (`None`, `Skip`)

---

## Setup

Install this extension in MakeCode via:




## 🧑‍💻 License

MIT License

---

Created by Robo Risen
For support or inquiries, contact: qna@roborisen.com
