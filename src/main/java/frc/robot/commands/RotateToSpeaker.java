// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// import frc.robot.Constants;
// import frc.robot.subsystems.drive.Drive;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class RotateToSpeaker extends ProfiledPIDCommand {
//   private final Drive drivetrain;
//   Field2d field = new Field2d();
//   Field2d bruh = new Field2d();

//   private static Rotation2d target = new Rotation2d();
//   private static Translation2d dist;

//   /** Creates a new RotateToSpeaker. */
//   public RotateToSpeaker(Drive drivetrain) {
//     super(
//         // The ProfiledPIDController used by the command
//         new ProfiledPIDController(
//             // The PID gains
//             2,
//             0,
//             0,
//             // The motion profile constraints
//             new TrapezoidProfile.Constraints(6, 6)),
//         // This should return the measurement
//         () -> drivetrain.getPose().getRotation().getRadians(),
//         // This should return the goal (can also be a constant)
//         () -> new TrapezoidProfile.State(target.getRadians(), 0.),
//         // This uses the output
//         (output, setpoint) -> {
//           // Create a ChassisSpeeds with the desired velocities
//           ChassisSpeeds chassisSpeeds = new ChassisSpeeds(output, 0, setpoint.velocity);

//           // Use the output (and setpoint, if desired) here
//           drivetrain.runVelocity(chassisSpeeds);
//           SmartDashboard.putNumber("setpoint", setpoint.velocity);
//         });
//     // Use addRequirements() here to declare subsystem dependencies.
//     // Configure additional PID options by calling `getController` here.
//     addRequirements(drivetrain);
//     getController().enableContinuousInput(-Math.PI, Math.PI);

//     this.drivetrain = drivetrain;
//   }

//   @Override
//   public void initialize() {
//     super.initialize();

//     Pose2d pose = drivetrain.getPose();
//     Pose2d target = Constants.SPEAKER_DISTANCE_TARGET;

//     Translation2d dist =
//         new Translation2d(pose.getX() - target.getX(), pose.getY() - target.getY());

//     Rotation2d angle = dist.getAngle();
//     RotateToSpeaker.dist = dist;
//     RotateToSpeaker.target = angle;

//     field.setRobotPose(new Pose2d(dist, dist.getAngle()));
//     SmartDashboard.putData("distnace", field);

//     bruh.setRobotPose(Constants.SPEAKER_DISTANCE_TARGET);
//     SmartDashboard.putData("speaker", bruh);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     // SmartDashboard.putNumber("target", target.getRadians());
//     // SmartDashboard.putNumber("heading",
//     // drivetrain.getState().Pose.getRotation().getRadians());
//     // SmartDashboard.putNumber("measuremtn", Math
//     // .abs(target.getRadians() -
//     // drivetrain.getRotation3d().toRotation2d().getRadians()));

//     return Math.abs(target.getRadians() - drivetrain.getPose().getRotation().getRadians()) < 0.1;
//   }
// }
