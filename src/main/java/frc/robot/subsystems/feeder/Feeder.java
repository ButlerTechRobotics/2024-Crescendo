// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.VendorWrappers.Neo;

/** Add your docs here. */
public class Feeder extends SubsystemBase {

    private DigitalInput FeederBeamBreak;
    /**
     * Motor object that controls the four axles on the front of the Feeder. 
     * A positive voltage spins the axles to suck a note into the robot.
     */
    private Neo frontFeederMotor;
    /**
     * Motor object that controls the two axles on the back of the Feeder.
     * A positive voltage spins the axles to suck a note into the robot.
     */
    private Neo backFeederMotor;

    private SimpleMotorFeedforward frontFeedforward;
    private SimpleMotorFeedforward backFeedforward;
    
    private PIDController frontController;
    private PIDController backController;

    public Feeder() {
        FeederBeamBreak = new DigitalInput(FeederConstants.FeederBeamBreakID);

        frontFeederMotor = new Neo("frontFeeder", FeederConstants.frontFeederMotorID);
        backFeederMotor = new Neo("backFeeder", FeederConstants.backFeederMotorID);

        frontFeedforward = new SimpleMotorFeedforward(
            FeederConstants.kSFrontFeederVolts,
            FeederConstants.kVFrontFeederVoltsPerRPM);

        backFeedforward = new SimpleMotorFeedforward(
            FeederConstants.kSBackFeederVolts, 
            FeederConstants.kVBackFeederVoltsPerRPM);

        frontController = new PIDController(
            FeederConstants.kPFrontFeederVoltsPerRPM, 0, 0);

        backController = new PIDController(
            FeederConstants.kPBackFeederVoltsPerRPM, 0, 0);
        
        configMotors();
    }

    public boolean isRingInFeeder() {
        return !FeederBeamBreak.get();
    }

    /**
     * Sets the RPM of the Feeder axles, using closed loop control.
     * @param rpm - RPM to have the motors spin at. A positive value will Feeder the note.
     */
    public void setRPM(double rpm) {
        double frontFeedforwardOutputVolts = frontFeedforward.calculate(rpm);
        double frontPIDOutputVolts = frontController.calculate(getFrontRPM(), rpm);
        frontFeederMotor.setVoltage(frontFeedforwardOutputVolts + frontPIDOutputVolts);

        double backFeedforwardOutputVolts = backFeedforward.calculate(rpm);
        double backPIDOutputVolts = backController.calculate(getBackRPM(), rpm);
        backFeederMotor.setVoltage(backFeedforwardOutputVolts + backPIDOutputVolts);

    }

    /**
     * Sets the voltage of both Feeder motors.
     * @param volts - Voltage to feed the Feeder motors. A positive value will Feeder a note.
     */
    public void setVolts(double volts) {
        frontFeederMotor.setVoltage(volts);
        backFeederMotor.setVoltage(volts);
    }

    public double getFrontRPM() {
        return frontFeederMotor.getVelocity();
    }

    public double getBackRPM() {
        return backFeederMotor.getVelocity();
    }

    private void configMotors() {
        frontFeederMotor.setInverted(false);
        backFeederMotor.setInverted(false);

        frontFeederMotor.setSmartCurrentLimit(50);
        backFeederMotor.setSmartCurrentLimit(50);

        frontFeederMotor.setIdleMode(IdleMode.kBrake);
        backFeederMotor.setIdleMode(IdleMode.kBrake);

        frontFeederMotor.setVelocityConversionFactor(1);
        backFeederMotor.setVelocityConversionFactor(1);

        frontFeederMotor.burnFlash();
        backFeederMotor.burnFlash();
    }


    @Override
    public void periodic() {
        Logger.recordOutput("Feeder/frontFeederMotorRPM", frontFeederMotor.getVelocity());
        Logger.recordOutput("Feeder/backFeederMotorRPM", backFeederMotor.getVelocity());
        Logger.recordOutput("Feeder/isRingInFeeder", isRingInFeeder());
    }

}
