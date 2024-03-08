// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

<<<<<<< HEAD
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.superstructure.shooter.Shooter;
// import frc.robot.util.AllianceFlipUtil;
// import java.util.function.Supplier;
// import org.littletonrobotics.junction.AutoLogOutput;

// /** A command that angles the arm from multi-distance position from the target. */
// public class MultiDistanceShot extends Command {
//   Supplier<Pose2d> poseSupplier;
//   Pose2d targetPose;
//   Shooter shooter;
//   InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

//   double distance;
//   double speed;

//   /**
//    * Creates a new MultiDistanceArm command.
//    *
//    * @param poseSupplier The supplier for the robot's current pose.
//    * @param targetPose The target pose to shoot at.
//    * @param shooter The arm subsystem.
//    */
//   public MultiDistanceShot(Supplier<Pose2d> poseSupplier, Pose2d targetPose, Shooter shooter) {
//     this.poseSupplier = poseSupplier;
//     this.targetPose = targetPose;
//     this.shooter = shooter;

//     // Populate the distance map with distance-angle pairs
//     distanceMap.put(1.0, 3000.);
//     distanceMap.put(2.3, 3500.);
//     distanceMap.put(4.0, 4000.);
//     distanceMap.put(4.9, 4500.);
//     distanceMap.put(6.2, 5000.);
//     distanceMap.put(7.5, 5500.);
//   }
=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** A command that angles the arm from multi-distance position from the target. */
public class MultiDistanceShot extends Command {
  Supplier<Pose2d> poseSupplier;
  Pose2d targetPose;
  Shooter shooter;
  InterpolatingDoubleTreeMap distanceMap = new InterpolatingDoubleTreeMap();

  double distance;
  double speedRPM;

  /**
   * Creates a new MultiDistanceArm command.
   *
   * @param poseSupplier The supplier for the robot's current pose.
   * @param targetPose The target pose to shoot at.
   * @param shooter The shooter subsystem.
   */
  public MultiDistanceShot(Supplier<Pose2d> poseSupplier, Pose2d targetPose, Shooter shooter) {
    this.poseSupplier = poseSupplier;
    this.targetPose = targetPose;
    this.shooter = shooter;

    // Populate the distance map with distance-angle pairs
    distanceMap.put(1.0, 2500.0);
    distanceMap.put(1.5, 2500.0);
    distanceMap.put(2.0, 3000.0);
    distanceMap.put(2.5, 3000.0);
    distanceMap.put(3.0, 4000.0);
    distanceMap.put(3.5, 4500.7);
    distanceMap.put(4.0, 5000.53);
    distanceMap.put(4.5, 5000.3);
    distanceMap.put(5.0, 5000.0);
    distanceMap.put(5.5, 5000.19);
    distanceMap.put(6.0, 5000.475);
  }
>>>>>>> Sonic

//   @Override
//   public void initialize() {
//     // Apply any necessary transformations to the target pose
//     targetPose = AllianceFlipUtil.apply(targetPose);
//   }

//   @Override
//   public void execute() {
//     // Calculate the distance from the current pose to the target pose
//     distance = poseSupplier.get().getTranslation().getDistance(targetPose.getTranslation());

<<<<<<< HEAD
//     // Get the corresponding angle from the distance-angle map
//     speed = distanceMap.get(distance);

//     // Run the flywheel at the calculated angle
//     shooter.setSetpoint(speed, speed);

//     // Put the distance on the SmartDashboard
//     SmartDashboard.putNumber("Distance", getDistance());
//   }
=======
    // Set the shooter goal based on the distance
    if (distance < 5) {
      shooter.setGoal(Shooter.Goal.SHOOTING);
    } else if (distance < 20) {
      shooter.setGoal(Shooter.Goal.SHOOTINGFAR);
    } else {
      shooter.setGoal(Shooter.Goal.IDLE);
    }

    // Put the distance on the SmartDashboard
    SmartDashboard.putNumber("Distance", getDistance());
  }

  @Override
  public void end(boolean interrupted) {
    // Sets the arm to home when the command ends
    shooter.setSetpoint(0.0, 0.0);
  }
>>>>>>> Sonic

//   @Override
//   public void end(boolean interrupted) {
//     // Sets the arm to home when the command ends
//     shooter.setSetpoint(0.0, 0.0);
//   }

//   @Override
//   public boolean isFinished() {
//     // The command never finishes on its own
//     return false;
//   }

<<<<<<< HEAD
//   /**
//    * Gets the distance from the current pose to the target pose.
//    *
//    * @return The distance in units.
//    */
//   @AutoLogOutput(key = "Shooter/DistanceToTarget")
//   public double getDistance() {
//     return distance;
//   }

//   /**
//    * Gets the angle of the arm.
//    *
//    * @return The angle in units per second.
//    */
//   @AutoLogOutput(key = "Shooter/Speed")
//   public double getSpeed() {
//     return speed;
//   }
// }
=======
  /**
   * Gets the angle of the arm.
   *
   * @return The angle in units per second.
   */
  @AutoLogOutput(key = "Shooter/SpeedRPM")
  public double getSpeed() {
    return speedRPM;
  }
}
>>>>>>> Sonic
