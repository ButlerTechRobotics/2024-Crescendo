// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.arm.ArmPositionPID;

public class PositionArmPID extends Command {
  /** Creates a new ArmEncoderPosition. */
  ArmPositionPID m_arm;

  double m_position;

  /** Creates a new Arm. */
  public PositionArmPID(ArmPositionPID arm, double position) {
    m_arm = arm;
    m_position = position;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setPosition(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setPosition(m_arm.getPostion());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
