package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs;

    /** PID controller for the top motor and flywheels.
     * The PID constants for this use a velocity of rotations per second
     * in order to be compatable with WPILib's PIDController class.
     */
    private PIDController topFlywheelsPID;

    /** PID controller for the bottom motor and flywheels.
     * The PID constants for this use a velocity of rotations per second
     * in order to be compatable with WPILib's PIDController class.
     */
    private PIDController bottomFlywheelsPID;

    /** Feedforward model for an individual set of flywheels. 
     * This assumes that the shooter's flywheel physics is 
     * symmetric on the top and bottom sides.
     * The feedforward constants for this use a velocity of rotations per second
     * in order to be compatable with WPILib's SimpleMotorFeedforward class.
     */
    private SimpleMotorFeedforward flywheelsFeedforward;


    public Shooter(ShooterIO io) {
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();
        
        topFlywheelsPID = new PIDController(
            ShooterConstants.kPFlywheelsVoltsSecondsPerMeter, 
            ShooterConstants.kIFlywheelsVoltsPerMeter, 
            ShooterConstants.kDFlywheelsVoltsSecondsSquaredPerMeter);

        bottomFlywheelsPID = new PIDController(
            ShooterConstants.kPFlywheelsVoltsSecondsPerMeter, 
            ShooterConstants.kIFlywheelsVoltsPerMeter, 
            ShooterConstants.kDFlywheelsVoltsSecondsSquaredPerMeter);


        flywheelsFeedforward = new SimpleMotorFeedforward(
            ShooterConstants.kSFlywheelsVolts,
            ShooterConstants.kVFlywheelsVoltsSecondsPerMeter,
            ShooterConstants.kAFlywheelsVoltsSecondsSquaredPerMeter);
    }

    /**
     * Sets the surface speed of the top set of flywheels.
     * This is theoretically the speed that this side of the note will have when exiting the shooter.
     * @param metersPerSecond - Desired surface speed in meters per second.
     */
    public void setTopFlywheelsMetersPerSecond(double metersPerSecond) {
        double feedforwardOutput = flywheelsFeedforward.calculate(metersPerSecond);
        double pidOutput = topFlywheelsPID.calculate(inputs.topFlywheelsMetersPerSecond, metersPerSecond);
        io.setTopMotorVolts(feedforwardOutput + pidOutput);
    }

    /**
     * @return - The shooter's top flywheel surface speed in meters per second
     */
    public double getTopFlywheelsMetersPerSecond() {
        return inputs.topFlywheelsMetersPerSecond;
    }

    /**
     * Sets the surface speed of the bottom set of flywheels.
     * This is theoretically the speed that this side of the note will have when exiting the shooter.
     * @param metersPerSecond - Desired surface speed in meters per second.
     */
    public void setBottomFlywheelsMetersPerSecond(double metersPerSecond) {
        double feedforwardOutput = flywheelsFeedforward.calculate(metersPerSecond);
        double pidOutput = bottomFlywheelsPID.calculate(inputs.bottomFlywheelsMetersPerSecond, metersPerSecond);
        io.setBottomMotorVolts(feedforwardOutput + pidOutput);
    }


    /**
     * Returns true if both flywheels are spinning within some threshold of their target speeds.
     * @param rpmThreshold - Max distance from the setpoint for this function to still return true, in meters per second.
     */
    public boolean flywheelsAtSetpoints(double topSetpointMetersPerSecond, double bottomSetpointMetersPerSecond, double thresholdMetersPerSecond) {

        return 
            Math.abs(topSetpointMetersPerSecond - inputs.topFlywheelsMetersPerSecond) < thresholdMetersPerSecond
                && Math.abs(bottomSetpointMetersPerSecond - inputs.bottomFlywheelsMetersPerSecond) < thresholdMetersPerSecond;
    }
    

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        
        Logger.processInputs("shooterInputs", inputs);

        Logger.recordOutput("shooter/topFlywheelsSetpoint", topFlywheelsPID.getSetpoint());
        Logger.recordOutput("shooter/bottomFlywheelsSetpoint", bottomFlywheelsPID.getSetpoint());

    };
}
