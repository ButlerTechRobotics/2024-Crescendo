// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.arm.Arm;

// SmartArm is a command that controls the arm based on the current drive mode and whether smart
// control is enabled.
public class SmartArm extends Command {
  // The arm subsystem this command will run on.
  Arm arm;

  // Creates a new SmartArm command that requires the given arm subsystem.
  public SmartArm(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled. Stops the arm.
  @Override
  public void initialize() {
    arm.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the current drive mode and whether smart control is enabled.
    DriveModeType driveModeType = SmartController.getInstance().getDriveModeType();
    boolean isSmartControlled = SmartController.getInstance().isSmartControlEnabled();

    // If the drive mode is SAFE, stop the arm and return.
    if (driveModeType == DriveModeType.SAFE) {
      arm.setArmTargetAngle(129);
      return;
    }

    // If smart control is enabled, and the drive mode is AMP, set the target
    // position of the arm to 80.
    if (isSmartControlled) {
      if (driveModeType == DriveModeType.AMP) {
        arm.setArmTargetAngle(190);
        return;
      }
      if (driveModeType == DriveModeType.SPEAKER || driveModeType == DriveModeType.FEED) {
        arm.setArmTargetAngle(SmartController.getInstance().getTargetAimingParameters().armAngle());
        return;
      }
    }
    // If drive mode is not AMP and the drive mode is SPEAKER or FEED, set the
    // target position of the arm based on the target aiming parameters.
    else if ((SmartController.getInstance().getDriveModeType() == DriveModeType.SPEAKER
            || SmartController.getInstance().getDriveModeType() == DriveModeType.FEED)
        && SmartController.getInstance().getTargetAimingParameters().effectiveDistanceToTarget()
            < SmartController.getInstance().getPrerollDistance()) {
      arm.setArmTargetAngle(SmartController.getInstance().getTargetAimingParameters().armAngle());
      return;
    }
  }

  // Called once the command ends or is interrupted. Currently does nothing.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end. Currently never ends.
  @Override
  public boolean isFinished() {
    return false;
  }
}
