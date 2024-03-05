// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.VendorWrappers.Neo;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private DigitalInput intakeProximitySwitch;
    /**
     * Motor object that controls the four axles on the intake of the intake.
     * A positive voltage spins the axles to suck a note into the robot.
     */
    private Neo intakeMotor;

    private SimpleMotorFeedforward intakeFeedforward;

    private PIDController intakeController;

    public Intake() {
        intakeProximitySwitch = new DigitalInput(IntakeConstants.intakeProximitySwitchID);

        intakeMotor = new Neo("Intake", IntakeConstants.intakeMotorID);

        intakeFeedforward = new SimpleMotorFeedforward(
                IntakeConstants.kSIntakeVolts,
                IntakeConstants.kVIntakeVoltsPerRPM);

        intakeController = new PIDController(
                IntakeConstants.kPIntakeVoltsPerRPM, 0, 0);

        configMotors();
    }

    public boolean isRingInIntake() {
        return !intakeProximitySwitch.get();
    }

    /**
     * Sets the RPM of the intake axles, using closed loop control.
     * 
     * @param rpm - RPM to have the motors spin at. A positive value will intake the
     *            note.
     */
    public void setRPM(double rpm) {
        double intakeFeedforwardOutputVolts = intakeFeedforward.calculate(rpm);
        double intakePIDOutputVolts = intakeController.calculate(getintakeRPM(), rpm);
        intakeMotor.setVoltage(intakeFeedforwardOutputVolts + intakePIDOutputVolts);
    }

    /**
     * Sets the voltage of both intake motors.
     * 
     * @param volts - Voltage to feed the intake motors. A positive value will
     *              intake a note.
     */
    public void setVolts(double volts) {
        intakeMotor.setVoltage(volts);
    }

    public double getintakeRPM() {
        return intakeMotor.getVelocity();
    }

    private void configMotors() {
        intakeMotor.setInverted(false);

        intakeMotor.setSmartCurrentLimit(40);

        intakeMotor.setIdleMode(IdleMode.kBrake);

        intakeMotor.setVelocityConversionFactor(1);

        intakeMotor.burnFlash();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("intake/IntakeMotorRPM", intakeMotor.getVelocity());
        Logger.recordOutput("intake/isRingInIntake", isRingInIntake());
    }

}
