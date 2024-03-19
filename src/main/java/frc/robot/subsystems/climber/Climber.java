// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private TalonFX leader = new TalonFX(23);
  private TalonFX follower = new TalonFX(24);

  /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
  private double targetPosition = 0;
  private final PositionVoltage m_positionVoltage =
      new PositionVoltage(
          targetPosition, targetPosition, false, targetPosition, 0, false, false, false);

  /** Creates a new flyWheelEncoder. */
  public Climber() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    leader.setNeutralMode(NeutralModeValue.Brake);
    follower.setControl(new Follower(leader.getDeviceID(), true));

    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    configs.Slot0.kP = 0.15; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
    // volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -10;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leader.getConfigurator().apply(configs);

      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void setPosition(double speed) {
    targetPosition = speed;
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public double getPosition() {
    return leader.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    leader.setControl(m_positionVoltage.withPosition(targetPosition));

    // This method will be called once per scheduler run
  }
}
