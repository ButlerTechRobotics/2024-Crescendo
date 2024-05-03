// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;
import frc.robot.util.vendorwrappers.Neo;

/**
 * Module IO implementation for SparkFlex drive motor controller, SparkFlex turn motor controller,
 * and CANcoder.
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIONeo implements ModuleIO {
  private final Neo driveSparkMax;
  private final Neo turnSparkMax;
  private final CANcoder cancoder;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final StatusSignal<Double> turnAbsolutePosition;

  private final Rotation2d absoluteEncoderOffset;

  public ModuleIONeo(ModuleConfig config) {

    driveSparkMax = new Neo(config.driveID());
    turnSparkMax = new Neo(config.turnID());
    cancoder = new CANcoder(config.absoluteEncoderChannel(), canbus);
    absoluteEncoderOffset = config.absoluteEncoderOffset(); // MUST BE CALIBRATED

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 11);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 12);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 101);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 102);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 103);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 104);

    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 15);
    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 16);
    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 17);
    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 205);
    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 206);
    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 207);
    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 208);
    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 209);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(config.turnMotorInverted());
    driveSparkMax.setSmartCurrentLimit(60);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    // driveSparkMax.setCANTimeout(0);
    // turnSparkMax.setCANTimeout(0);

    // //Wait 1 second before flashing NEO eprom memory
    // try {
    //   Thread.sleep(1000);
    // } catch (Exception e) {}

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();

    cancoder.getConfigurator().apply(new CANcoderConfiguration());
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnAbsolutePosition.setUpdateFrequency(10.0);
    cancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / moduleConstants.driveReduction();
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / moduleConstants.driveReduction();
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    turnAbsolutePosition.refresh();
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(
            turnRelativeEncoder.getPosition() / moduleConstants.turnReduction());
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / moduleConstants.turnReduction();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
