// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.magazine;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Magazine extends SubsystemBase {
  private final MagazineIO magazineIO;
  private final MagazineIOInputsAutoLogged inputs = new MagazineIOInputsAutoLogged();
  private double targetVoltage = 0;
  private boolean isShooting = false;

  /** Creates a new Magazine. */
  public Magazine(MagazineIO magazineIO) {
    this.magazineIO = magazineIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    magazineIO.updateInputs(inputs);
    Logger.processInputs("Magazine", inputs);
    isShooting();
  }

  public void forward() {
    setSpeedRPM(6);
    isShooting = false;
  }

  public void shoot() {
    setSpeedRPM(7);
    isShooting = true;
  }

  public void backward() {
    setSpeedRPM(-7);
    isShooting = false;
  }

  public void slowForward() {
    setSpeedRPM(1.8);
    isShooting = false;
  }

  public void slowBackward() {
    setSpeedRPM(-1.8);
    isShooting = false;
  }

  public void stop() {
    magazineIO.stop();
    isShooting = false;
  }

  @AutoLogOutput(key = "Magazine/isShooting")
  public boolean isShooting() {
    return isShooting;
  }

  public void setSpeedRPM(double speedRPM) {
    targetVoltage = speedRPM;
    magazineIO.runVoltage(targetVoltage, targetVoltage);
  }

  @AutoLogOutput(key = "Magazine/TargetRPM")
  public double getTargetRPM() {
    return targetVoltage;
  }

  @AutoLogOutput(key = "Magazine/TopVelocityRPM")
  public double topVelocityRPM() {
    // Convert Radians per second to Meters per second
    return inputs.topVelocityRPM;
  }

  @AutoLogOutput(key = "Magazine/BottomVelocityRPM")
  public double bottomVelocityRPM() {
    // Convert Radians per second to Meters per second
    return inputs.bottomVelocityRPM;
  }
}
