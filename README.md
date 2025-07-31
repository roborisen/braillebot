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

## 🧑‍💻 License

MIT License

---

Created by Robo Risen
For support or inquiries, contact: qna@roborisen.com
