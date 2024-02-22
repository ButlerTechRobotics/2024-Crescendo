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
  InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();
  // Map to hold distance-angle pairs

  double distance; // Distance from the current pose to the target pose
  double targetAngle; // The angle to set the arm to
  double angle;

  /*
     * Constructor for the MultiDistanceArm command.
  =======
    double distance;
    double angle;

    /**
     * Creates a new MultiDistanceShot command.
  >>>>>>> b05b67a (Auton Path and working arm distance (Arm positions need tuned))
  =======
    double distance;
    double angle;

    /**
     * Creates a new MultiDistanceShot command.
  >>>>>>> 369a2a5ecb2539ba6897e38a3373fa7779399062
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
    distanceMap.put(2.3, -2.0);
    distanceMap.put(3.6, -4.0);
    distanceMap.put(4.9, -6.0);
    distanceMap.put(6.2, -8.0);
    distanceMap.put(7.5, -10.0);
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

    // Get the corresponding speed from the distance-speed map
    angle = distanceMap.get(distance);

    // Run the flywheel at the calculated speed
    armPID.setPosition(angle);

    // Get the corresponding speed from the distance-speed map
    angle = distanceMap.get(distance);

    // Run the flywheel at the calculated speed
    armPID.setPosition(angle);
  }

  @Override
  public void end(boolean interrupted) {

    // Reset the arm position when the command ends
    armPID.setPosition(0);

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
   * <<<<<<< HEAD <<<<<<< HEAD Gets the current target angle of the arm. ======= Gets the speed of
   * the flywheel. >>>>>>> b05b67a (Auton Path and working arm distance (Arm positions need tuned))
   * ======= Gets the speed of the flywheel. >>>>>>> 369a2a5ecb2539ba6897e38a3373fa7779399062
   *
   * @return The target angle in degrees.
   */
  @AutoLogOutput(key = "Arm/Angle")
  public double getSpeed() {
    return angle;
  }
}
