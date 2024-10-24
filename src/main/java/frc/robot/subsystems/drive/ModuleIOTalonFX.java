// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;
  private final StatusSignal<Double> driveTemp;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;
  private final StatusSignal<Double> turnTemp;

  private final Rotation2d absoluteEncoderOffset;

  // Controller Configs
  private final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnTalonConfig = new TalonFXConfiguration();
  private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

  public ModuleIOTalonFX(ModuleConfig config) {
    driveTalon = new TalonFX(config.driveID(), canbus);
    turnTalon = new TalonFX(config.turnID(), canbus);
    cancoder = new CANcoder(config.absoluteEncoderChannel(), canbus);
    absoluteEncoderOffset = config.absoluteEncoderOffset();

    // Config Motors
    driveTalonConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
    driveTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveTalonConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    driveTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    turnTalonConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    turnTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnTalonConfig.MotorOutput.Inverted =
        config.turnMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    for (int i = 0; i < 4; i++) {
      boolean statusOK = driveTalon.getConfigurator().apply(driveTalonConfig, 0.1) == StatusCode.OK;
      setDriveBrakeMode(true);
      statusOK =
          statusOK && turnTalon.getConfigurator().apply(turnTalonConfig, 0.1) == StatusCode.OK;
      setTurnBrakeMode(true);
      statusOK =
          statusOK
              && cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.1)
                  == StatusCode.OK;
      if (statusOK) break;
    }

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();
    driveTemp = driveTalon.getDeviceTemp();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();
    turnTemp = turnTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.odometryFrequency, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveTemp,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnTemp);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveTemp,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnTemp);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble())
            / moduleConstants.driveReduction();
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble())
            / moduleConstants.driveReduction();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};
    inputs.driveTemp = driveTemp.getValueAsDouble();

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / moduleConstants.turnReduction());
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / moduleConstants.turnReduction();
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
    inputs.turnTemp = turnTemp.getValueAsDouble();

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble(
                (Double value) ->
                    Units.rotationsToRadians(value) / moduleConstants.driveReduction())
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(
                (Double value) -> Rotation2d.fromRotations(value / moduleConstants.turnReduction()))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    brakeModeExecutor.execute(
        () -> {
          synchronized (driveTalonConfig) {
            driveTalonConfig.MotorOutput.NeutralMode =
                enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            driveTalon.getConfigurator().apply(driveTalonConfig, 0.25);
          }
        });
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    brakeModeExecutor.execute(
        () -> {
          synchronized (turnTalonConfig) {
            turnTalonConfig.MotorOutput.NeutralMode =
                enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            turnTalon.getConfigurator().apply(turnTalonConfig, 0.25);
          }
        });
  }
}
