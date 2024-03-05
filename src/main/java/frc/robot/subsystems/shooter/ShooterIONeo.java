package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VendorWrappers.Neo;

public class ShooterIONeo implements ShooterIO {
    // Hardware
    public Neo topMotor;
    public Neo bottomMotor;
    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder;

    // Controllers
    private SparkPIDController topController;
    private SparkPIDController bottomController;
    // Open loop
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

    public ShooterIONeo() {
        // Init Hardware
        topMotor = new Neo(ShooterConstants.topMotorID);
        bottomMotor = new Neo(ShooterConstants.bottomMotorID);
        configMotors();

        topEncoder = topMotor.getEncoder();
        bottomEncoder = bottomMotor.getEncoder();

        // Reset encoders
        topEncoder.setPosition(0.0);
        bottomEncoder.setPosition(0.0);
        topEncoder.setMeasurementPeriod(10);
        bottomEncoder.setMeasurementPeriod(10);
        topEncoder.setAverageDepth(2);
        bottomEncoder.setAverageDepth(2);

        // Get controllers
        topController = topMotor.getPIDController();
        bottomController = bottomMotor.getPIDController();
        setPID(gains.kP(), gains.kI(), gains.kD());
        setFF(gains.kS(), gains.kV(), gains.kA());
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topPositionRads = Units.rotationsToRadians(topEncoder.getPosition()) / reduction;
        inputs.topVelocityRpm = topEncoder.getVelocity() / reduction;
        inputs.topAppliedVolts = topMotor.getAppliedOutput();
        inputs.topOutputCurrent = topMotor.getOutputCurrent();
        inputs.topTempCelsius = topMotor.getMotorTemperature();

        inputs.bottomPositionRads = Units.rotationsToRadians(bottomEncoder.getPosition()) / reduction;
        inputs.bottomVelocityRpm = bottomEncoder.getVelocity() / reduction;
        inputs.bottomAppliedVolts = bottomMotor.getAppliedOutput();
        inputs.bottomOutputCurrent = bottomMotor.getOutputCurrent();
        inputs.bottomTempCelsius = bottomMotor.getMotorTemperature();
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
    public void runVolts(double topVolts, double bottomVolts) {
        topMotor.setVoltage(-topVolts);
        bottomMotor.setVoltage(-bottomVolts);
    }

    @Override
    public void runVelocity(double topFlywheelRPM, double bottomFlywheelRPM) {
        topController.setReference(
                topFlywheelRPM * reduction,
                CANSparkBase.ControlType.kVelocity,
                0,
                ff.calculate(topFlywheelRPM),
                SparkPIDController.ArbFFUnits.kVoltage);
        bottomController.setReference(
            bottomFlywheelRPM * reduction,
                CANSparkBase.ControlType.kVelocity,
                0,
                ff.calculate(bottomFlywheelRPM),
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        topController.setP(kP);
        topController.setI(kI);
        topController.setD(kD);
        bottomController.setP(kP);
        bottomController.setI(kI);
        bottomController.setD(kD);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        ff = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void runCharacterizationTopVolts(double volts) {
        topMotor.setVoltage(volts);
    }

    @Override
    public void runCharacterizationBottomVolts(double volts) {
        bottomMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }
}