// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.arm.ArmPositionPID;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** A command that shoots game piece from multi-distance position from the target. */
public class MultiDistanceArm extends Command {
  Supplier<Pose2d> poseSupplier;
  Pose2d targetPose;
  ArmPositionPID armPID;
  InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

  double distance;
  double targetAngle;

  /**
   * Creates a new MultiDistanceArm command.
   *
   * @param poseSupplier The supplier for the robot's current pose.
   * @param targetPose The target pose to aim at.
   * @param armPID The arm subsystem.
   */
  public MultiDistanceArm(Supplier<Pose2d> poseSupplier, Pose2d targetPose, ArmPositionPID armPID) {
    this.poseSupplier = poseSupplier;
    this.targetPose = targetPose;
    this.armPID = armPID;

    // Populate the distance map with distance-angle pairs
    distanceMap.put(1.0, 0.0);
    distanceMap.put(2.3, 20.0);
    distanceMap.put(3.6, 40.0);
    distanceMap.put(4.9, 60.0);
    distanceMap.put(6.2, 80.0);
    distanceMap.put(7.5, 100.0);
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

    // Get the corresponding angle from the distance-speed map
    targetAngle = distanceMap.get(distance);

    // Run the shooter at the calculated speed
    armPID.setPosition(targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the shooter when the command ends
    armPID.setPosition(0);
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
   * Gets the speed of the shooter.
   *
   * @return The speed in units per second.
   */
  @AutoLogOutput(key = "Arm/Angle")
  public double getAngle() {
    return targetAngle;
  }
}
