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
          new Rotation3d(0, Math.toRadians(-28.), Math.toRadians(150.)));

  public static final Transform3d ROBOT_TO_CAMERA_BR =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.625),
              Units.inchesToMeters(-11.5),
              Units.inchesToMeters(9.5)),
          new Rotation3d(0, Math.toRadians(-27.), Math.toRadians(-150.)));

  public static final Transform3d ROBOT_TO_CAMERA_BACK =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-3.0), Units.inchesToMeters(0.0), Units.inchesToMeters(14.75)),
          new Rotation3d(0, Math.toRadians(-20.), Math.toRadians(180.)));
}
