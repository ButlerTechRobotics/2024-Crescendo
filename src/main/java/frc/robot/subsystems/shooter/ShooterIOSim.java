// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim topSim =
      new FlywheelSim(DCMotor.getNeoVortex(1), reduction, 0.00363458292);
  private final FlywheelSim bottomSim =
      new FlywheelSim(DCMotor.getNeoVortex(1), reduction, 0.00363458292);

  private final PIDController topController = new PIDController(gains.kP(), gains.kI(), gains.kD());
  private final PIDController bottomController =
      new PIDController(gains.kP(), gains.kI(), gains.kD());
  private SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(gains.kS(), gains.kV(), gains.kA());

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  private Double topSetpointRPM = null;
  private Double bottomSetpointRPM = null;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    topSim.update(0.02);
    bottomSim.update(0.02);
    // control to setpoint
    if (topSetpointRPM != null && bottomSetpointRPM != null) {
      runVolts(
          topController.calculate(topSim.getAngularVelocityRPM(), topSetpointRPM)
              + ff.calculate(topSetpointRPM),
          bottomController.calculate(bottomSim.getAngularVelocityRPM(), bottomSetpointRPM)
              + ff.calculate(bottomSetpointRPM));
    }

    inputs.topPositionRads += Units.radiansToRotations(topSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.topVelocityRPM = topSim.getAngularVelocityRPM();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topOutputCurrent = topSim.getCurrentDrawAmps();

    inputs.bottomPositionRads +=
        Units.radiansToRotations(bottomSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.bottomVelocityRPM = bottomSim.getAngularVelocityRPM();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomOutputCurrent = bottomSim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double topVolts, double bottomVolts) {
    topAppliedVolts = MathUtil.clamp(topVolts, -12.0, 12.0);
    bottomAppliedVolts = MathUtil.clamp(bottomVolts, -12.0, 12.0);
    topSim.setInputVoltage(topAppliedVolts);
    bottomSim.setInputVoltage(bottomAppliedVolts);
  }

  @Override
  public void runVelocity(double topRpm, double bottomRpm) {
    topSetpointRPM = topRpm;
    bottomSetpointRPM = bottomRpm;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    topController.setPID(kP, kI, kD);
    bottomController.setPID(kP, kI, kD);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    ff = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void stop() {
    runVelocity(0.0, 0.0);
  }

  @Override
  public void runCharacterizationTopVolts(double volts) {
    topSetpointRPM = null;
    bottomSetpointRPM = null;
    runVolts(volts, 0.0);
  }

  @Override
  public void runCharacterizationBottomVolts(double volts) {
    topSetpointRPM = null;
    bottomSetpointRPM = null;
    runVolts(0.0, volts);
  }
}
