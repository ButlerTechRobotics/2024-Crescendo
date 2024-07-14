// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.gamepieceVision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class GamepieceVisionCameraConstants {
  public static final Transform3d ROBOT_TO_CAMERA_NOTES =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.625), Units.inchesToMeters(11.5), Units.inchesToMeters(9.5)),
          new Rotation3d(0, Math.toRadians(-28.), Math.toRadians(150.)));
}
