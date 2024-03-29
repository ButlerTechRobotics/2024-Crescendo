// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.arm.ArmPositionPID;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** A command that angles the arm from multi-distance position from the target. */
public class MultiDistanceArm extends Command {
  Supplier<Pose2d> poseSupplier;
  Pose2d targetPose;
  ArmPositionPID armPID;
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
  public MultiDistanceArm(Supplier<Pose2d> poseSupplier, Pose2d targetPose, ArmPositionPID armPID) {
    this.poseSupplier = poseSupplier;
    this.targetPose = targetPose;
    this.armPID = armPID;

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
    armPID.setPosition(angle);

    // Put the distance on the SmartDashboard
    SmartDashboard.putNumber("Distance", getDistance());
  }

  @Override
  public void end(boolean interrupted) {
    // Sets the arm to home when the command ends
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
   * Gets the angle of the arm.
   *
   * @return The angle in units per second.
   */
  @AutoLogOutput(key = "Arm/Angle")
  public double getSpeed() {
    return angle;
  }
}
