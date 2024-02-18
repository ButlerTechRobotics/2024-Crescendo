// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Import necessary libraries and packages
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.arm.ArmPositionPID;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** A command that controls the arm position based on the distance from the target. */
public class MultiDistanceArm extends Command {
  // Declare variables
  Supplier<Pose2d> poseSupplier; // Supplier for the robot's current pose
  Pose2d targetPose; // The target pose to aim at
  ArmPositionPID armPID; // The arm subsystem
  InterpolatingDoubleTreeMap distanceMap =
      new InterpolatingDoubleTreeMap(); // Map to hold distance-angle pairs

  double distance; // Distance from the current pose to the target pose
  double targetAngle; // The angle to set the arm to

  /**
   * Constructor for the MultiDistanceArm command.
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

    // Log the distance to the Shuffleboard
    SmartDashboard.putNumber("Distance", distance);

    // Get the corresponding angle from the distance-angle map
    targetAngle = distanceMap.get(distance);

    // Set the arm position to the calculated angle
    armPID.setPosition(targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    // Reset the arm position when the command ends
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
   * Gets the current target angle of the arm.
   *
   * @return The target angle in degrees.
   */
  @AutoLogOutput(key = "Arm/Angle")
  public double getAngle() {
    return targetAngle;
  }
}
