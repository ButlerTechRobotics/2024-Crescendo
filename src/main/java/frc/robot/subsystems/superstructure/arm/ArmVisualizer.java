// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ArmVisualizer {
  private final Mechanism2d mechanism;
  private final MechanismLigament2d arm;
  private final String key;

  public static Translation2d armOrigin = new Translation2d(-0.238, 0.298);
  public static double armLength = 28.0;

  public ArmVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    MechanismRoot2d root = mechanism.getRoot("pivot", 1.0, 0.4);
    arm = new MechanismLigament2d("arm", 28, 20.0, 6, new Color8Bit(color));
    root.append(arm);
  }

  /** Update arm visualizer with current arm angle */
  public void update(double angleRads) {
    arm.setAngle(Rotation2d.fromRadians(angleRads));
    Logger.recordOutput("Arm/" + key + "Mechanism2d", mechanism);

    // Log 3d poses
    Pose3d pivot =
        new Pose3d(armOrigin.getX(), 0.0, armOrigin.getY(), new Rotation3d(0.0, -angleRads, 0.0));
    Logger.recordOutput("Arm/" + key + "3d", pivot);
  }
}
