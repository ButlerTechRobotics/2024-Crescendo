// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

/** Add your docs here. */
public class ReverseIntake extends Command {


    private Intake intake;
    private Feeder feeder;

    public ReverseIntake(Intake intake, Feeder feeder) {
        this.intake = intake;
        this.feeder = feeder;

        addRequirements(intake, feeder);
    }

    @Override
    public void initialize() {
        intake.setVolts(-8);
        feeder.setVolts(-8);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intake.setVolts(0);
        feeder.setVolts(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
