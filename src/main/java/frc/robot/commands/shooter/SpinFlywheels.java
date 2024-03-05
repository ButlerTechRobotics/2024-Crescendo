package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class SpinFlywheels extends Command {

    private Shooter shooter; 
    private double topFlywheelMetersPerSecond;
    private double bottomFlywheelMetersPerSecond;   

    /** Spins the shooter flywheels up to a desired surface speed, in meters per second. This does not launch the note. */
    public SpinFlywheels(double topFlywheelMetersPerSecond, double bottomFlywheelMetersPerSecond, Shooter shooter) {
        this.shooter = shooter;
        this.topFlywheelMetersPerSecond = topFlywheelMetersPerSecond;
        this.bottomFlywheelMetersPerSecond = bottomFlywheelMetersPerSecond;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.setTopFlywheelsMetersPerSecond(topFlywheelMetersPerSecond);
        shooter.setBottomFlywheelsMetersPerSecond(bottomFlywheelMetersPerSecond);
    }

    @Override
    public void end(boolean isInterrupted) {
    }

    @Override
    public boolean isFinished() {
        return shooter.flywheelsAtSetpoints(topFlywheelMetersPerSecond, bottomFlywheelMetersPerSecond, 0.5);
    }
    
}
