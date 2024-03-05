package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants.ArmConstants;
import frc.robot.VendorWrappers.Neo;

public class ArmIONeo implements ArmIO {

    private Neo motor;
    private AbsoluteEncoder armAbsoluteEncoder;

    public ArmIONeo() {

        /** Absolute Encoder config */

        /** Motor config */
        motor = new Neo("armMotor", ArmConstants.motorID);
        configMotors();

        armAbsoluteEncoder = motor.getAbsoluteEncoder();
        configEncoders();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {

        inputs.armVelocityDegreesPerSecond = armAbsoluteEncoder.getVelocity() * 360;

        inputs.armEncoderReadingDegrees = armAbsoluteEncoder.getPosition() * 360;
        inputs.armAngleDegrees = inputs.armEncoderReadingDegrees;

        // getBusVoltage() gets voltage fed into motor controller, getAppliedOutput()
        // gets a percent the motor is running at.
        // found this solution on chiefdelphi
        inputs.armMotorAppliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();

        if (inputs.armAngleDegrees <= ArmConstants.armMinAngleDegrees) {
            inputs.atLowerLimit = true;
        } else if (inputs.armAngleDegrees >= ArmConstants.armMaxAngleDegrees) {
            inputs.atUpperLimit = true;
        } else {
            inputs.atLowerLimit = false;
            inputs.atUpperLimit = false;
        }
    }

    @Override
    public void setArmMotorVolts(double volts) {
        motor.setVoltage(volts);
    }

    // @Override
    // public void setArmEncoderPosition(double degrees) {
    // absoluteEncoder.setPosition(degrees/360.);
    // }

    private void configMotors() {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(80);
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);
        motor.burnFlash();
    }

    private void configEncoders() {
        armAbsoluteEncoder.setPositionConversionFactor(0.5);
        armAbsoluteEncoder.setZeroOffset(0.0);
    }

}
