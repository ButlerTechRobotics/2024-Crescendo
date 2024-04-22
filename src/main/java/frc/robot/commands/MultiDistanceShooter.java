// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** A command that spins the shooter from multi-distance position from the target. */
public class MultiDistanceShooter extends Command {
  Supplier<Pose2d> poseSupplier;
  Shooter shooter;
  InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

  double distance;
  double rpm;

  Translation2d orignalPose;
  Translation2d targetPose;

  /**
   * Creates a new MultiDistanceShooter command.
   *
   * @param poseSupplier The supplier for the robot's current pose.
   * @param targetPose The target pose to shoot at.
   * @param shooter The shooter subsystem.
   */
  public MultiDistanceShooter(
      Supplier<Pose2d> poseSupplier, Translation2d targetPose, Shooter shooter) {
    this.poseSupplier = poseSupplier;
    this.shooter = shooter;
    this.orignalPose = targetPose;

    // Populate the distance map with distance-angle pairs
    distanceMap.put(1.0, 3000.0);
    distanceMap.put(1.5, 3000.0);
    distanceMap.put(2.0, 3200.0);
    distanceMap.put(2.5, 3500.0);
    distanceMap.put(3.0, 3800.0);
    distanceMap.put(3.5, 4250.0); // 4000 4/4/2024 3:56PM
    distanceMap.put(3.75, 4250.0); // 4000 4/4/2024 3:56PM
    distanceMap.put(4.0, 4400.0); // 4200 4/4/2024 3:56PM
    distanceMap.put(4.2, 4400.0); // 4250 4/4/2024 3:56PM
    distanceMap.put(4.5, 4400.0); // 4300 4/4/2024 3:56PM
    distanceMap.put(5.0, 4400.0);
    distanceMap.put(5.5, 4500.0);
    distanceMap.put(6.0, 4750.0);
    distanceMap.put(6.5, 5000.0);
    distanceMap.put(7.0, 5500.0);
    distanceMap.put(8.0, 6500.0);
  }

  @Override
  public void initialize() {
    this.targetPose = AllianceFlipUtil.apply(orignalPose);
    // Apply any necessary transformations to the target pose
  }

  @Override
  public void execute() {
    // Calculate the distance from the current pose to the target pose
    distance = poseSupplier.get().getTranslation().getDistance(targetPose);

    // Get the corresponding angle from the distance-angle map
    rpm = distanceMap.get(distance);

    // Run the flywheel at the calculated angle
    shooter.setSetpoint(rpm, rpm);

    // Put the distance on the SmartDashboard
    SmartDashboard.putNumber("Distance", getDistance());
  }

  @Override
  public void end(boolean interrupted) {
    // Sets the shooter to 0 rpm when the command ends
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    // The command never finishes on its own
    return false;
  }

  /**
   * Gets the distance from the current pose to the target pose.
   *
   * @return The distance in units.
   */
  @AutoLogOutput(key = "Shooter/Distance")
  public double getDistance() {
    Logger.recordOutput("Shooter/Distance", distance);
    return distance;
  }

  /**
   * Gets the speed of the shooter.
   *
   * @return The speed in rotations per minute.
   */
  @AutoLogOutput(key = "Shooter/RPM")
  public double getSpeed() {
    return rpm;
  }
}
