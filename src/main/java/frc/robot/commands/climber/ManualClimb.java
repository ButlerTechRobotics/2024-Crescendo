// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberLeft;
import frc.robot.subsystems.climber.ClimberRight;

// ManualClimb is a command that sets the position of both the left and right climbers.
public class ManualClimb extends Command {
  // The left and right climber subsystems
  ClimberLeft climberLeft;
  ClimberRight climberRight;

  // The target position for the climbers
  double targetPosition;

  // Constructor for the ManualClimb command
  public ManualClimb(ClimberLeft climberLeft, ClimberRight climberRight, double targetPosition) {
    // Initialize the climber subsystems and target position
    this.climberLeft = climberLeft;
    this.climberRight = climberRight;
    this.targetPosition = targetPosition;

    // Declare subsystem dependencies
    addRequirements(climberLeft, climberRight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Nothing to do here for this command
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set the position of both climbers to the target position
    climberLeft.setPosition(targetPosition);
    climberRight.setPosition(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set the position of both climbers to their current position
    // This effectively stops the climbers from moving
    climberLeft.setPosition(climberLeft.getPosition());
    climberRight.setPosition(climberRight.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command never ends on its own
    return false;
  }
}
