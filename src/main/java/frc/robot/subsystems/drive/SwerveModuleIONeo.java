package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.VendorWrappers.Neo;

public class SwerveModuleIONeo implements SwerveModuleIO {
  private CANcoder absoluteEncoder;
  private Neo angleMotor;
  private Neo driveMotor;

  /**
   * @param driveMotorID - ID of the drive motor
   * @param angleMotorID - ID of the angle motor
   * @param angleOffsetDegrees - Offset of the angle motor, in degrees
   * @param cancoderID - ID of the absolute CANcoder mounted ontop of the swerve
   * @param isDriveMotorOnTop - Is drive motor mounted on top
   * @param isAngleMotorOnTop - Is angle motor mounted on top
   */
  public SwerveModuleIONeo(
      int driveMotorID,
      int angleMotorID,
      double angleOffsetDegrees,
      int cancoderID,
      boolean isDriveMotorOnTop,
      boolean isAngleMotorOnTop,
      String name) {

    /* Angle Encoder Config */
    absoluteEncoder = new CANcoder(cancoderID, "rio");
    configCANCoder(angleOffsetDegrees);

    /* Angle Motor Config */
    angleMotor = new Neo(name + "Steer", angleMotorID);
    if (isAngleMotorOnTop) {
      configAngleMotor(false);
    } else {
      configAngleMotor(true);
    }

    /* Drive Motor Config */
    driveMotor = new Neo(name + "Drive", driveMotorID);
    if (isDriveMotorOnTop) {
      configDriveMotor(false);
    } else {
      configDriveMotor(true);
    }
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.drivePositionMeters =
        driveMotor.getPosition()
            * (SwerveModuleConstants.driveGearReduction
                * SwerveModuleConstants.wheelCircumferenceMeters);
    inputs.driveVelocityMetersPerSecond =
        driveMotor.getVelocity()
            * (SwerveModuleConstants.driveGearReduction
                * SwerveModuleConstants.wheelCircumferenceMeters);
    inputs.angleAbsolutePositionDegrees =
        absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360;

    inputs.driveAppliedVoltage = driveMotor.getBusVoltage();
    inputs.driveCurrent = driveMotor.getOutputCurrent();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  @Override
  public void setAngleVoltage(double volts) {
    angleMotor.setVoltage(volts);
  }

  private void configCANCoder(double angleOffsetDegrees) {
    CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
    cancoderConfigs.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderConfigs.MagnetSensor.MagnetOffset = angleOffsetDegrees;
    cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    absoluteEncoder.getConfigurator().apply(cancoderConfigs);
  }

  private void configDriveMotor(boolean invertedValue) {
    // Neo is automatically reset to factory defaults upon construction
    driveMotor.setSmartCurrentLimit(MotorConstants.driveContinuousCurrentLimit);
    driveMotor.setInverted(invertedValue);
    driveMotor.setIdleMode(MotorConstants.driveNeutralMode);
    driveMotor.burnFlash();
  }

  private void configAngleMotor(boolean invertedValue) {
    // Neo is automatically reset to factory defaults upon construction
    angleMotor.setSmartCurrentLimit(MotorConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(invertedValue);
    angleMotor.setIdleMode(MotorConstants.angleNeutralMode);
    angleMotor.burnFlash();
  }
}
