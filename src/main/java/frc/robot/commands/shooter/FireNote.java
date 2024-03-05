package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;

public class FireNote extends Command {
    
    private Feeder feeder;
    private Timer timer;

    /** Pulses the indexer wheels to bring a note from the indexer into the shooter. */
    public FireNote(Feeder feeder) {
        this.feeder = feeder;

        timer = new Timer();
    }

    @Override
    public void initialize() {
        feeder.setVolts(12);
        timer.restart();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean isInterrupted) {
        feeder.setVolts(0);
    }
    
    @Override
    public boolean isFinished() {
        return timer.get() > 1.0;
    }
}
