package frc.robot.subsystems.rollers;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

/** Generic roller IO implementation for a roller or series of rollers using a SPARK Flex. */
public abstract class GenericRollerSystemIOSparkFlex implements GenericRollerSystemIO {
  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;

  private final double reduction;

  public GenericRollerSystemIOSparkFlex(
      int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    motor = new CANSparkFlex(id, CANSparkBase.MotorType.kBrushless);

    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

    motor.setSmartCurrentLimit(currentLimitAmps);
    motor.setInverted(invert);
    motor.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);

    encoder = motor.getEncoder();
  }

  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition()) / reduction;
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.outputCurrent = motor.getOutputCurrent();
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
