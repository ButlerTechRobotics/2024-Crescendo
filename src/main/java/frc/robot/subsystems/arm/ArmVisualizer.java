// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ArmVisualizer {
  private final String logKey;

  private final Mechanism2d mechanism;
  private final MechanismRoot2d mechanismRoot;
  private final MechanismLigament2d fixedLigament;
  private final MechanismLigament2d armLigament;

  double armLength = Units.inchesToMeters(18);

  public ArmVisualizer(String logKey, Color8Bit color) {
    this.logKey = logKey;
    mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
    mechanismRoot = mechanism.getRoot("Arm", 1.31115, 1.13081);
    fixedLigament =
        mechanismRoot.append(
            new MechanismLigament2d("Fixed", 0.63881, 90, 6, new Color8Bit(Color.kBlack)));
    armLigament =
        fixedLigament.append(
            new MechanismLigament2d("Arm", armLength, 90, 4, new Color8Bit(Color.kDarkBlue)));
  }

  public void update(double armAngle) {
    armLigament.setAngle(Units.radiansToDegrees(armAngle));

    Logger.recordOutput("Mechanism2d/" + logKey, mechanism);
    // Transform3d hardpoint =
    //     new Transform3d(
    //         Units.inchesToMeters(12.25), 0, Units.inchesToMeters(25.15), new Rotation3d(0, 0,
    // 0));

    // Pose3d armPose = new Pose3d(Units.inchesToMeters(10), 0, 0, new Rotation3d(0,
    // armAngle, 0));
    Pose3d armPose = getArmPose(armAngle);

    Logger.recordOutput("Mechanism3d/" + logKey, armPose);
  }

  Translation2d armRoot = new Translation2d(-0.31, 0.64);

  public Pose3d getArmPose(double armAngle) {
    return new Pose3d(armRoot.getX(), 0, armRoot.getY(), new Rotation3d(0, -armAngle, 0));
  }
}
