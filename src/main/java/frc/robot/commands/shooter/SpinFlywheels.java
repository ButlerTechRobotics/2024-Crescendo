package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class SpinFlywheels extends Command {

    private Shooter shooter; 
    private double topFlywheelRPM;
    private double bottomFlywheelRPM;   

    /** Spins the shooter flywheels up to a desired surface speed, in meters per second. This does not launch the note. */
    public SpinFlywheels(double topFlywheelRPM, double bottomFlywheelRPM, Shooter shooter) {
        this.shooter = shooter;
        this.topFlywheelRPM = topFlywheelRPM;
        this.bottomFlywheelRPM = bottomFlywheelRPM;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.setSetpoint(topFlywheelRPM, bottomFlywheelRPM);
    }

    @Override
    public void end(boolean isInterrupted) {
    }

    @Override
    public boolean isFinished() {
        return shooter.flywheelsAtSetpoints(topFlywheelRPM, bottomFlywheelRPM, 0.5);
    }
    
}
