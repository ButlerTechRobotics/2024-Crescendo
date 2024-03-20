// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.*;
import org.photonvision.PhotonCamera;

public class VisionPoseEstimator extends SubsystemBase {
  private static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(0.855, -0.1225, 0.23),
          new Rotation3d(0, (-24.0) / 360 * 2 * (Math.PI), 0));
  private final AprilTag estimator = new AprilTag(new PhotonCamera("FrontCamera"), ROBOT_TO_CAMERA);

  private final Notifier allNotifier =
      new Notifier(
          () -> {
            estimator.run();
          });

  public void init() {
    allNotifier.setName("runAll");
    allNotifier.startPeriodic(0.02);
  }

  private static Vector<N3> confidenceCalculator(AprilTag estimator) {
    double sum =
        Drive.getPose()
            .getTranslation()
            .getDistance(
                estimator.grabLatestEstimatedPose().estimatedPose.toPose2d().getTranslation());
    int tagCount = estimator.grabLatestEstimatedPose().targetsUsed.size();
    return VecBuilder.fill(
        0.005 * Math.pow(sum / tagCount, 2) / tagCount,
        0.005 * Math.pow(sum / tagCount, 2) / tagCount,
        0.001 * Math.pow(sum / tagCount, 2) / tagCount);
  }

  public void periodic() {
    if (estimator != null && estimator.grabLatestEstimatedPose() != null) {
      Drive.addVisionMeasurement(
          estimator.grabLatestEstimatedPose().estimatedPose.toPose2d(),
          estimator.grabLatestEstimatedPose().timestampSeconds,
          confidenceCalculator(estimator));
    }
  }
}
