// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeWheelsIO wheelsIO;

  private final IntakeWheelsIOInputsAutoLogged wheelsInputs = new IntakeWheelsIOInputsAutoLogged();
  private double targetSpeed = 0;
  private boolean intakeRequest = false;

  /** Creates a new Intake. */
  public Intake(IntakeWheelsIO wheelsIO) {
    this.wheelsIO = wheelsIO;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    wheelsIO.updateInputs(wheelsInputs);
    Logger.processInputs("IntakeWheels", wheelsInputs);
  }

  public void intake() {
    setSpeedRPM(40);
  }

  public void intakeSlow() {
    setSpeedRPM(20);
  }

  public void outtake() {
    setSpeedRPM(-50);
  }

  /** Stops the intake. */
  public void stop() {
    wheelsIO.stop();
  }

  public void enableIntakeRequest() {
    this.intakeRequest = true;
  }

  public void disableIntakeRequest() {
    this.intakeRequest = false;
  }

  @AutoLogOutput(key = "Intake/IntakeRequest")
  public boolean getIntakeRequest() {
    return intakeRequest;
  }

  public void setSpeedRPM(double speedRPM) {
    targetSpeed = speedRPM;
    wheelsIO.setSpeedRPM(targetSpeed);
  }

  /** Returns the current velocity in Rot Per Sec. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return wheelsInputs.velocityRPM;
  }

  @AutoLogOutput(key = "Intake/TargetSpeed")
  public double getTargetRot() {
    return targetSpeed;
  }

  public boolean atTargetSpeed() {
    return Math.abs(wheelsInputs.velocityRPM - getTargetRot()) < 0.5;
  }

  public enum IntakePositions {
    FLOOR,
    UP
  }
}
