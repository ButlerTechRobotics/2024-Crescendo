// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.vision.AprilTagVision;

public class Candle extends SubsystemBase {

  CANdle candle;
  AprilTagVision aprilTags;

  boolean intakeRequest = false;
  boolean hasGamePiece = false;
  boolean safeMode = false;
  boolean speakerMode = false;
  int stripLength = 75;
  int startOffset = 0;

  /** Creates a new LedController. */
  public Candle(AprilTagVision aprilTags) {
    candle = new CANdle(32);
    // candle.configLEDType(LEDStripType.GRB);
    // candle.configV5Enabled(true);
    this.aprilTags = aprilTags;
  }

  public void setHasGamePiece(boolean hasGamePiece) {
    this.hasGamePiece = hasGamePiece;
  }

  public void setIntakeRequest(boolean intakeRequest) {
    this.intakeRequest = intakeRequest;
  }

  public void setSafeMode(boolean safeMode) {
    this.safeMode = safeMode;
  }

  public void setSpeakerMode(boolean speakerMode) {
    this.speakerMode = speakerMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {

      // WHITE FOR 0 TAGS
      if (aprilTags.getPoseEstimationCount() == 0 || DriverStation.getAlliance().isEmpty()) {
        candle.animate(new SingleFadeAnimation(0, 0, 0, 255, 0.3, stripLength, startOffset));
        return;
      }
      // RED FOR 1 TAG
      if (aprilTags.getPoseEstimationCount() == 1 || DriverStation.getAlliance().isEmpty()) {
        candle.animate(new SingleFadeAnimation(228, 3, 3, 0, 0.3, stripLength, startOffset));
        return;
      }
      // ORANGE FOR 2 TAGS
      if (aprilTags.getPoseEstimationCount() == 2 || DriverStation.getAlliance().isEmpty()) {
        candle.animate(new SingleFadeAnimation(255, 140, 0, 0, 0.3, stripLength, startOffset));
        return;
      }
      // YELLOW FOR 3 TAGS
      if (aprilTags.getPoseEstimationCount() == 3 || DriverStation.getAlliance().isEmpty()) {
        candle.animate(new SingleFadeAnimation(255, 237, 0, 0, 0.3, stripLength, startOffset));
        return;
      }
      // GREEN FOR 4 TAGS
      if (aprilTags.getPoseEstimationCount() == 4 || DriverStation.getAlliance().isEmpty()) {
        candle.animate(new SingleFadeAnimation(0, 128, 38, 0, 0.3, stripLength, startOffset));
        return;
      }
      // BLUE FOR 5 TAGS
      if (aprilTags.getPoseEstimationCount() == 5 || DriverStation.getAlliance().isEmpty()) {
        candle.animate(new SingleFadeAnimation(0, 77, 255, 0, 0.3, stripLength, startOffset));
        return;
      }
      // PURPLE FOR 6 TAGS
      if (aprilTags.getPoseEstimationCount() == 6 || DriverStation.getAlliance().isEmpty()) {
        candle.animate(new SingleFadeAnimation(117, 7, 135, 0, 0.3, stripLength, startOffset));
        return;
      }
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        candle.animate(
            new LarsonAnimation(150, 0, 0, 0, 0.05, stripLength, BounceMode.Front, 7, 0));
      } else {
        candle.animate(
            new LarsonAnimation(0, 0, 150, 0, 0.05, stripLength, BounceMode.Front, 7, 0));
      }
      return;
    }
    // boolean isSpeakerMode = SmartController.getInstance().getDriveModeType() ==
    // DriveModeType.SPEAKER;

    boolean isSafeMode = SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE;
    if (isSafeMode) {
      candle.setLEDs(0, 0, 200);
    }

    if (hasGamePiece) {
      candle.animate(new StrobeAnimation(0, 255, 0, 0, 1, stripLength, startOffset));
    } else {
      candle.setLEDs(0, 0, 200);
    }

    if (intakeRequest) {
      candle.animate(new RainbowAnimation(255, 2, stripLength));
    } else {
      candle.setLEDs(0, 0, 200);
    }
  }
}