package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beambreak.BeamBreak;
import frc.robot.subsystems.magazine.Magazine;

public class FeedShooter extends Command {
  Magazine magazine;
  BeamBreak beamBreak;
  Timer timer;
  Timer shooterTimer;
  double shootDelay;

  public FeedShooter(
      Magazine magazine, BeamBreak beambreak, double shootDelay) {
    this.magazine = magazine;
    this.beamBreak = beambreak;
    this.shootDelay = shootDelay;
    addRequirements(magazine, beambreak);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    boolean isShooting = false;
    double realShootDelay = shootDelay;

    if (shooterTimer.hasElapsed(realShootDelay)) {
      magazine.shoot();
      isShooting = true;
    }

    Logger.recordOutput("FeedShooter/IsShooting", isShooting);
  }
}
