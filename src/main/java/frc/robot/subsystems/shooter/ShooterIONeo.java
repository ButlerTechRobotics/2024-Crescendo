package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VendorWrappers.Neo;

public class ShooterIONeo implements ShooterIO {

    public Neo topMotor;
    public Neo bottomMotor;

    public ShooterIONeo() {

        /** top motor config */
        topMotor = new Neo(ShooterConstants.topMotorID);

        /** bottom motor config */
        bottomMotor = new Neo(ShooterConstants.bottomMotorID);

        configMotors();

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topFlywheelsMetersPerSecond = topMotor.getVelocity()
                * ShooterConstants.flywheelCircumferenceMeters;
        inputs.bottomFlywheelsMetersPerSecond = bottomMotor.getVelocity()
                * ShooterConstants.flywheelCircumferenceMeters;
    }

    private void configMotors() {
        topMotor.restoreFactoryDefaults();
        topMotor.setInverted(false);
        topMotor.setIdleMode(IdleMode.kBrake);
        topMotor.setSmartCurrentLimit(40);
        topMotor.burnFlash();

        bottomMotor.restoreFactoryDefaults();
        bottomMotor.setInverted(false);
        bottomMotor.setIdleMode(IdleMode.kBrake);
        bottomMotor.setSmartCurrentLimit(40);
        bottomMotor.burnFlash();
    }

    @Override
    public void setTopMotorVolts(double volts) {
        if (topMotor.getMotorTemperature() > ShooterConstants.motorMaxTempCelsius) {
            volts = 0;
        }

        topMotor.setVoltage(volts);
    }

    @Override
    public void setBottomMotorVolts(double volts) {
        if (bottomMotor.getMotorTemperature() > ShooterConstants.motorMaxTempCelsius) {
            volts = 0;
        }

        bottomMotor.setVoltage(volts);
    }
}
