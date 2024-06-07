# Line Following Vehicle

Controls an autonomous color line-following vehicle that picks up and places boxes based on box color and size, as well as navigating an obstacle avoidance course. This robot utilizes PID control with an IR sensor array, three ultrasonic sensors, four color sensors, four independantly controlled motors, 2 servos, and a button.

## Getting Started

This will not go through how to assemble the robot, but just the methods of implementing the code.

### Code Prerequisites

Requirements for the software and other tools to build, test and push 
- [Arduino IDE](https://www.arduino.cc/en/software)

### Robot Material Prerequisites

#### Electrical

- [**4x** TCS230 TCS3200 RGB Light Color Recognition Sensor](https://www.amazon.com/DEVMO-TCS3200-Recognition-Detector-Compatible/dp/B07Y88WRNQ/ref)
- [**3x** HC-SR04 Ultrasonic Sensor](https://www.amazon.com/Dorhea-Ultrasonic-Distance-Duemilanove-Rapsberry/dp/B07L68X65N/ref?th=1)
- [**1x** Teensy 4.1 Microcontroller](https://www.pjrc.com/store/teensy41.html)
- [**2x** L298 Dual H Bridge Motor Speed Controller DC 6.5V-27V 7A PWM](https://www.amazon.com/DROK-Controller-Regulator-Industrial-Optocoupler/dp/B06XGD5SCB?th=1)
- [**4x** 116 RPM Planetary Gear Motor](https://www.servocity.com/116-rpm-premium-planetary-gear-motor/)
- [**1x** 2000 Series Dual Mode Servo (25-3, Speed)](https://www.servocity.com/2000-series-dual-mode-servo-25-3-speed/)
- [**1x** 2000 Series Dual Mode Servo (25-2, Torque)](https://www.servocity.com/2000-series-dual-mode-servo-25-2/)
- [**1x** 6x6x4.3mm Pushbutton](https://www.amazon.com/Gikfun-6x6x5mm-Switch-Button-Arduino/dp/B00R17XUFC)
- [**1x** QTR-8RC Reflectance Sensor Array](https://www.amazon.com/dp/B0C1C31YWJ?psc=1&ref=ppx_yo2ov_dt_b_product_details)

### Mechanical

- [2x Metal Mounting Brackets](https://www.servocity.com/1116-series-grid-plate-17-x-29-hole-136-x-232mm/)
- [4x Omni Wheels](https://www.servocity.com/3604-series-omni-wheel-14mm-bore-96mm-diameter/)
- Various Fasteners

### Installing

Download the main folder, and with Arduino IDE installed open the `.ino` file. Verify the port is selected to the Teensy 4.1 and if you are connected to a microcontroller, the port you've selected. You can find more information regarding basic Arduino handling online.

## Running the code

To use the preset tests, use the `.run()` method of each class inside of the `void loop`:

### Obstacle Avoidance

```cpp
void loop(){
  obstacleAvoidance.run()
}
```

### Pickup & Place

```cpp
void loop(){
  pickupPlace.run()
}
```

### Calibrations

There are three calibrations that are required based on the lighting and components used. The first is the 

## Development

This project is structured in a slightly unconventional way. As this is a robot where small tweaks have been made throughout it's lifecycle and not a solidified end product, there are very few `private` objects inside of the classes, instead allowing the developer to modify parameters on the fly.

Additionally, there are very few constructors for the classes, specifically to make the `Initialization.h` file as easy to read for debugging purposes.

### Structure

```
├── main
│   ├── BoxControl.h
│   ├── Initialization.h
│   ├── LineFollowing.h
│   ├── Motion.h
│   ├── ObstacleAvoidance.h
│   ├── PickupPlace.h
│   ├── main.ino
│   ├── controls
│   │   ├── PIDController.h
│   │   └── Utils.h
│   └── sensors
│       ├── Button.h
│       ├── ColorSensor.h
│       ├── IRSensorArray.h
│       ├── MWServo.h
│       ├── Motor.h
│       └── UltraSonic.h
└── readme.md
```

### File Descriptions

`main/`
- `BoxControl.h`: Defines a class, box, which keeps information regarding the box's attributes like color and size, as well as the methods required for handling the box, like grabbing, picking up, etc.
- `Initialization.h`: Defines the pins for the sensors, calibration points, initializes sensors, etc.
- `LineFollowing.h`: Methods on line following, such as PID control, centering a robot on a parallel & perpendicular line.
- `Motion.h`: Creates functions to move the robot in cardinal directions, rotate the robot, and translate it a specified distance with calibration points.
- `ObstacleAvoidance.h`: A state machine that has the overarching logic on how to navigate the obstacle course.
- `PickupPlace.h`: Defines the methods to systematically go through the coruse and pick up and place the box while following a line.
- `main.ino`: The main Arduino file where the setup and loop functions are defined. The directory and the file name must be the same due to Arduino's conventions.

`main/sensors/` Houses generalized sensor logic
- `Button.h`: Class for a simple pushbutton toggle
- `ColorSensor.h`: Class for the TCS230 TCS3200 RGB Light Color Sensor. Includes a moving average to filter out erroneous color readings, and an algorithm to determine color based on calibration points and euclidean distance.
- `IRSensorArray.h`: Class for the IR Array that controls the PID system. Includes a moving average for the output values.
- `MWServo.h`: This builds upon the pre-made arduino `Servo.h` folder by allowing for variable speed of the motors.
- `Motor.h`: Determines the logic for controlling the four motors on the bottom of the robot, utilizing calibration points to allow the developer to determine % speed, % pwm, and absolute speed.
- `UltraSonic.h`: Provides methods for reading the distance from the ultrasonic sensors.

`.vscode/`: 
- `c_cpp_properties.json`: Sets the path for vscode to import the base `Arduino.h` and `Servo.h` files so IntelliSense works correctly. These paths will need to be changed on your computer if you want VScode to display IntelliSense.

`.VSCodeCounter/`: Merely displays stats about lines of code & comments written.

## Built With

  - [Contributor Covenant](https://www.contributor-covenant.org/) - Used
    for the Code of Conduct
  - [Creative Commons](https://creativecommons.org/) - Used to choose
    the license

## Authors

  - [**Max Westerman**](maxwesterman.com)
  - **Billie Thompson** - *Provided README Template* -

## License

This project is licensed under the [CC0 1.0 Universal](LICENSE.md)
Creative Commons License - see the [LICENSE.md](LICENSE.md) file for
details

## Acknowledgments

My groupmates who helped me build the robot:
  - Danny Carey
  - Charlie Butrick

