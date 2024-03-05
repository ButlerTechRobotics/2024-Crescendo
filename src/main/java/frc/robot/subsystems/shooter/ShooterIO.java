package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        /**
         * Linear surface speed of the top set of flywheels.
         */
        public double topFlywheelsMetersPerSecond = 0.0;
        /**
         * Linear surface speed of the bottom set of flywheels.
         */
        public double bottomFlywheelsMetersPerSecond = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {};

    /** Run the shooter's top motor at the specified volts. */
    public default void setTopMotorVolts(double volts) {};

    /** Run the shooter's bottom motor at the specified volts. */
    public default void setBottomMotorVolts(double volts) {};
}
