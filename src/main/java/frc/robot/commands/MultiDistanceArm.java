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
import frc.robot.subsystems.superstructure.arm.ArmPositionPID;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** A command that angles the arm from multi-distance position from the target. */
public class MultiDistanceArm extends Command {
  Supplier<Pose2d> poseSupplier;
  ArmPositionPID armPID;
  InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

  double distance;
  double angle;

  Translation2d orignalPose;
  Translation2d targetPose;

  /**
   * Creates a new MultiDistanceArm command.
   *
   * @param poseSupplier The supplier for the robot's current pose.
   * @param targetPose The target pose to shoot at.
   * @param armPID The arm subsystem.
   */
  public MultiDistanceArm(
      Supplier<Pose2d> poseSupplier, Translation2d targetPose, ArmPositionPID armPID) {
    this.poseSupplier = poseSupplier;
    this.armPID = armPID;
    this.orignalPose = targetPose;

    // Populate the distance map with distance-angle pairs
    distanceMap.put(1.5, 4.0 + 0.7); // 3.5
    distanceMap.put(2.0, 12.02 + 0.7);
    distanceMap.put(2.5, 18.4 + 0.7);
    distanceMap.put(3.0, 21.7 + 0.7);
    distanceMap.put(3.5, 24.98 + 0.7);
    distanceMap.put(4.0, 29.85 + 0.7); // 29.85
    distanceMap.put(4.5, 31.00 + 0.7);
    distanceMap.put(5.0, 34.00 + 0.7);
    distanceMap.put(5.5, 34.50 + 0.7);
    distanceMap.put(6.0, 35.00);
    // distanceMap.put(6.5, 30.875);
    // distanceMap.put(7.0, 31.375);
    // distanceMap.put(8.0, 31.175);
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
    angle = distanceMap.get(distance);

    // Run the flywheel at the calculated angle
    armPID.setPosition(angle);

    // Put the distance on the SmartDashboard
    SmartDashboard.putNumber("Distance", getDistance());
  }

  @Override
  public void end(boolean interrupted) {
    // Sets the arm to home when the command ends
    armPID.setPosition(2.5);
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
  @AutoLogOutput(key = "Arm/Distance")
  public double getDistance() {
    Logger.recordOutput("Arm/Distance", distance);
    return distance;
  }

  /**
   * Gets the angle of the arm.
   *
   * @return The angle in units per second.
   */
  @AutoLogOutput(key = "Arm/Angle")
  public double getAngle() {
    return angle;
  }
}
