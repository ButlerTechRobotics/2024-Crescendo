package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.*;


public class ShooterIOSim implements ShooterIO {
    private FlywheelSim leftSim;
    private FlywheelSim rightSim;

    public ShooterIOSim() {

        leftSim = new FlywheelSim(
            DCMotor.getKrakenX60(1),
            ShooterConstants.flywheelGearReduction,
            0.000374);

        rightSim = new FlywheelSim(
            DCMotor.getKrakenX60(1),
            ShooterConstants.flywheelGearReduction,
            0.000374);
            
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftSim.update(0.02);
        rightSim.update(0.02);

        inputs.topFlywheelsMetersPerSecond = leftSim.getAngularVelocityRPM()/60.*ShooterConstants.flywheelCircumferenceMeters;
        inputs.bottomFlywheelsMetersPerSecond = rightSim.getAngularVelocityRPM()/60.*ShooterConstants.flywheelCircumferenceMeters;
    }

    @Override
    public void setTopMotorVolts(double volts) {
        leftSim.setInputVoltage(volts);
    }

    @Override
    public void setBottomMotorVolts(double volts) {
        rightSim.setInputVoltage(volts);
    }
}
