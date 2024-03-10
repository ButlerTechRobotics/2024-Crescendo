// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.arm.ArmPositionPID;

import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * A command that angles the arm from multi-distance position from the target.
 */
public class MultiDistanceArm extends Command {
  private final Supplier<Pose2d> poseSupplier;
  private final Pose2d redSpeakerPose;
  private final Pose2d blueSpeakerPose;
  private final ArmPositionPID armPID;
  private Pose2d targetPose;
  private InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

  private double distance;
  private double angle;

  /**
   * Creates a new MultiDistanceArm command.
   *
   * @param poseSupplier    The supplier for the robot's current pose.
   * @param redSpeakerPose  The pose of the red speaker.
   * @param blueSpeakerPose The pose of the blue speaker.
   * @param armPID          The arm subsystem.
   */
  public MultiDistanceArm(Supplier<Pose2d> poseSupplier, Pose2d redSpeakerPose, Pose2d blueSpeakerPose,
      ArmPositionPID armPID) {
    this.poseSupplier = poseSupplier;
    this.redSpeakerPose = redSpeakerPose;
    this.blueSpeakerPose = blueSpeakerPose;
    this.armPID = armPID;

    // Populate the distance map with distance-angle pairs
    distanceMap.put(1.0, 0.0);
    distanceMap.put(1.5, 10.88);
    distanceMap.put(2.0, 14.00);
    distanceMap.put(2.5, 17.5);
    distanceMap.put(3.0, 21.0);
    distanceMap.put(3.5, 22.12);
    distanceMap.put(4.0, 22.93);
    // distanceMap.put(4.5, 25.3);
    // distanceMap.put(5.0, 28.0);
    // distanceMap.put(5.5, 31.19);
    // distanceMap.put(6.0, 34.475);
  }

  @Override
  public void initialize() {
    // Get the alliance color
    Optional<DriverStation.Alliance> colorOptional = DriverStation.getAlliance();

    // Check if the alliance color is present
    if (colorOptional.isPresent()) {
      DriverStation.Alliance color = colorOptional.get();

      // Select the target pose based on the alliance color
      if (color == DriverStation.Alliance.Red) {
        targetPose = redSpeakerPose;
      } else if (color == DriverStation.Alliance.Blue) {
        targetPose = blueSpeakerPose;
      }
    } else {
      // Handle the case where the alliance color is not available
      // For example, you could set the target pose to a default value
    }
  }

  @Override
  public void execute() {
    // Calculate the distance from the current pose to the target pose
    distance = poseSupplier.get().getTranslation().getDistance(targetPose.getTranslation());

    // Get the corresponding angle from the distance-angle map
    angle = distanceMap.get(distance) + 1.5;

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
