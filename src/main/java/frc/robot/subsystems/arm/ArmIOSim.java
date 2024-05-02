// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  SingleJointedArmSim armSim;

  TalonFX armMotor;
  TalonFXSimState armMotorSim;

  CANcoder armEncoder;

  public ArmIOSim() {
    armMotor = new TalonFX(50);
    armMotorSim = armMotor.getSimState();

    armEncoder = new CANcoder(52);

    armMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;

    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    armEncoder.getConfigurator().apply(armEncoderConfig);

    var armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var slot0Configs = armConfig.Slot0;
    slot0Configs.kP = 6;
    slot0Configs.kI = 0.2;
    slot0Configs.kD = 1;
    slot0Configs.kS = 0.4;
    slot0Configs.kV = 0.01;
    slot0Configs.kA = 0.001;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    armConfig.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    armConfig.Feedback.SensorToMechanismRatio = 1;
    armConfig.Feedback.RotorToSensorRatio = 10;

    var motionMagicConfig = armConfig.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = 0.5;
    motionMagicConfig.MotionMagicAcceleration = 1;
    motionMagicConfig.MotionMagicJerk = 5;

    armMotor.getConfigurator().apply(armConfig);

    armSim =
        new SingleJointedArmSim(
            DCMotor.getNeoVortex(1),
            10,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(20), Units.lbsToKilograms(23)),
            Units.inchesToMeters(20),
            Units.degreesToRadians(-175),
            Units.degreesToRadians(175),
            false,
            Units.degreesToRadians(90));
  }

  public void updateInputs(ArmIOInputs inputs) {
    armSim.setInput(armMotorSim.getMotorVoltage());
    armSim.update(0.02);
    armEncoder.getSimState().setRawPosition(armSim.getAngleRads() / (Math.PI * 2));
    armEncoder.getSimState().setVelocity(armSim.getVelocityRadPerSec() / (Math.PI * 2));

    inputs.armAbsolutePositionDeg = armEncoder.getPosition().refresh().getValue() * Math.PI * 2;
    inputs.armRelativePositionDeg = armEncoder.getPosition().refresh().getValue() * Math.PI * 2;
    inputs.armVelocityRadPerSec = armEncoder.getVelocity().refresh().getValue();
    inputs.armCurrentAmps = new double[] {armMotorSim.getSupplyCurrent()};
    inputs.armTempCelcius = new double[] {armMotor.getDeviceTemp().getValue()};
  }

  public void setArmTarget(double target) {
    var control = new PositionVoltage(0);
    armMotor.setControl(
        control.withPosition(Units.radiansToRotations(target)).withSlot(0).withEnableFOC(true));
  }

  public void setBrakeMode(boolean armBrake) {
    armMotor.setNeutralMode(armBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
