// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class CameraConstants {
  public static final Transform3d ROBOT_TO_CAMERA_BL =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.625), Units.inchesToMeters(11.5), Units.inchesToMeters(9.5)),
          new Rotation3d(0, Math.toRadians(-18.), Math.toRadians(210.)));

  public static final Transform3d ROBOT_TO_CAMERA_BR =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.625),
              Units.inchesToMeters(-11.5),
              Units.inchesToMeters(9.5)),
          new Rotation3d(0, Math.toRadians(-18.), Math.toRadians(-210.)));

  public static final Transform3d ROBOT_TO_CAMERA_BACK =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-3.0), Units.inchesToMeters(0.0), Units.inchesToMeters(14.75)),
          new Rotation3d(0, Math.toRadians(-20.), Math.toRadians(180.)));
}
