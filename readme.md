<p align="center">
<img src="./docs/images/Github%20Header.png" width="500">


<p align="center">
  <a href="https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.2.1"><img src="https://img.shields.io/badge/WPILib-2024.2.1-AC2B37" /></a>
  <a href="https://v6.docs.ctr-electronics.com/en/stable/"><img src="https://img.shields.io/badge/Phoenix6-24.1.0-97D700"></a>
  <a href="https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information"><img src="https://img.shields.io/badge/REVLib-2024.2.0-f05a28"></a>
</p>

<p align="center">This is the code used by Butler Tech Robotics, teams 325 and 144 during their 2024 season. The code is based off of (insert AK link here) It provides support for swerve drivetrains, as well as April Tag field tracking with Limelight and full robot simulation with code replay.

## What is AdvantageKit?
<ul>
<li>AdvantageKit is a logging, telemetry, and replay framework developed by Team 6328. AdvantageKit enables log replay, where the full state of the robot code can be replayed in simulation based on a log file.</li>
<li>AdvantageKit is intended to be used with (AdvantageScope).
  </ul>

## What is AdvantageScope?
<ul>
<li>AdvantageScope is a robot diagnostics, log review/analysis, and data visualization application for FIRST Robotics Competition teams.</li>

</ul>

When deploying code to your robot, you'll need to change the value of `currentMode` to `Mode.REAL` in `Constants.java`

## Other values to adjust

### Drive.java
  * `MAX_LINEAR_SPEED` , `TRACK_WIDTH_X` , `TRACK_WIDTH_Y`

### GyroIOPigeon2.java
  * `pigeon`

### Module.java
  * `WHEEL_RADIUS`

### ModuleIOTalonFX.java or ModuleIOSparkMax.java
  * `DRIVE_GEAR_RATIO`, `TURN_GEAR_RATIO`, `IS_TURN_MOTOR_INVERTED`, along with all drive, turn, CANcoder, CANbus, and `absoluteEncoderOffsets` values



## Credits

This project is heavily based on example code from Team 5712 - Hemlock's Gray Matter from their [AdvantageKitSwerveTemplate repository](https://github.com/Hemlock5712/AdvantageKitSwerveTemplate)
