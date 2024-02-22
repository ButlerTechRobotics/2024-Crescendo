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
  double angle;

  /**
   * Creates a new MultiDistanceShot command.
   *
   * @param poseSupplier The supplier for the robot's current pose.
   * @param targetPose The target pose to shoot at.
   * @param armPID The flywheel subsystem.
   */
  public MultiDistanceArm(Supplier<Pose2d> poseSupplier, Pose2d targetPose, ArmPositionPID armPID) {
    this.poseSupplier = poseSupplier;
    this.targetPose = targetPose;
    this.armPID = armPID;

    // Populate the distance map with distance-speed pairs
    distanceMap.put(1.0, 0.0);
    distanceMap.put(2.3, 2.0);
    distanceMap.put(3.6, 4.0);
    distanceMap.put(4.9, 6.0);
    distanceMap.put(6.2, 8.0);
    distanceMap.put(7.5, 10.0);
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

    // Get the corresponding speed from the distance-speed map
    angle = distanceMap.get(distance);

    // Run the flywheel at the calculated speed
    armPID.setPosition(angle);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the flywheel when the command ends
    armPID.setPosition(0.0);
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
   * Gets the speed of the flywheel.
   *
   * @return The speed in units per second.
   */
  @AutoLogOutput(key = "Arm/Angle")
  public double getSpeed() {
    return angle;
  }
}
