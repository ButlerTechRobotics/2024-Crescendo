// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.superstructure.climber.Climber;

// public class PositionClimbPID extends Command {
//   /** Creates a new ClimbEncoderPosition. */
//   Climber m_climb;

//   double m_position;

//   /** Creates a new Climb. */
//   public PositionClimbPID(Climber Climb, double position) {
//     m_climb = Climb;
//     m_position = position;
//     addRequirements(m_climb);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_climb.setPosition(m_position);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_climb.setPosition(m_climb.getPosition());
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
