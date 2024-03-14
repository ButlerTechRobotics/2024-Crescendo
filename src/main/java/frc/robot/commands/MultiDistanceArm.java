// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** A command that angles the arm from multi-distance position from the target. */
public class MultiDistanceArm extends Command {
  Supplier<Pose2d> poseSupplier;
  Pose2d targetPose;
  Arm arm;
  InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

  double distance;
  double angle;

  /**
   * Creates a new MultiDistanceArm command.
   *
   * @param poseSupplier The supplier for the robot's current pose.
   * @param targetPose The target pose to shoot at.
   * @param armPID The arm subsystem.
   */
  public MultiDistanceArm(Supplier<Pose2d> poseSupplier, Pose2d targetPose, Arm arm) {
    this.poseSupplier = poseSupplier;
    this.targetPose = targetPose;
    this.arm = arm;

    // Populate the distance map with distance-angle pairs
    distanceMap.put(1.0, 0.0);
    distanceMap.put(1.5, 0.0);
    distanceMap.put(2.0, 5.5);
    distanceMap.put(2.5, 7.0);
    distanceMap.put(3.0, 8.15);
    distanceMap.put(3.5, 8.7);
    distanceMap.put(4.0, 9.53);
    distanceMap.put(4.5, 10.3);
    distanceMap.put(5.0, 11.0);
    distanceMap.put(5.5, 11.19);
    distanceMap.put(6.0, 11.475);
  }

  @Override
  public void initialize() {
    // Apply any necessary transformations to the target pose
    targetPose = AllianceFlipUtil.apply(targetPose);
  }

  @Override
  public void execute() {
    // Calculate the distance from the current pose to the target pose
    distance = poseSupplier.get().getTranslation().getDistance(targetPose.getTranslation());

    // Get the corresponding angle from the distance-angle map
    angle = distanceMap.get(distance);

    // Run the flywheel at the calculated angle
    arm.setDesiredDegrees(angle);

    // Put the distance on the SmartDashboard
    SmartDashboard.putNumber("Distance", getDistance());
  }

  @Override
  public void end(boolean interrupted) {
    // Sets the arm to home when the command ends
    arm.setDesiredDegrees(0.0);
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
  @AutoLogOutput(key = "Arm/DistanceToTarget")
  public double getDistance() {
    return distance;
  }

  /**
   * Gets the angle of the arm.
   *
   * @return The angle in units per second.
   */
  @AutoLogOutput(key = "Arm/Angle")
  public double getSpeed() {
    return angle;
  }
}
