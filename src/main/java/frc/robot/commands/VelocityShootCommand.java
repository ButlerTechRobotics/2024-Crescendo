// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class VelocityShootCommand extends Command {

  private final Shooter m_shooter;
  private final Timer m_timer;
  private final double m_shooterVelocitySet = 40.0;

  public VelocityShootCommand(Shooter shooter) {
    m_shooter = shooter;
    m_timer = new Timer();

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.setVelocity(m_shooterVelocitySet);
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    var m_shooterVelocity = m_shooter.getUpShooterTalonFX().getVelocity().getValue();
    if (m_timer.get() > 0.5 && m_shooterVelocity > m_shooterVelocitySet * 0.9) {}
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setVelocity(0.0);
  }
}
