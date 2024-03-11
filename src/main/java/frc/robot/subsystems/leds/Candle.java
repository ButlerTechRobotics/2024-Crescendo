// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase {
  private final CANdle candle;
  private final int NUM_LEDS = 75;
  private Color color;

  enum Color {
    GREEN(0, 254, 0),
    PURPLE(118, 0, 254),
    ORANGE(254, 55, 0),
    BLUE(0, 0, 254),
    WHITE(254, 254, 254),
    RED(254, 0, 0),
    OFF(0, 0, 0);

    public int r;
    public int g;
    public int b;

    Color(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  /** Creates a new light. */
  public Candle() {
    candle = new CANdle(32, "rio");
    candle.configBrightnessScalar(.5);
    candle.configLEDType(LEDStripType.GRB);
    setColor(Color.BLUE);
  }

  public void setColor(Color color) {
    this.color = color;
    candle.animate(null);
    candle.setLEDs(this.color.r, this.color.g, this.color.b);
  }

  public void setNoColor() {
    setColor(Color.OFF);
  }

  public Command setColorGreenCommand() {
    return runOnce(() -> setColor(Color.GREEN));
  }

  public Command setColorRespawnIdle() {
    return runOnce(() -> setColor(Color.BLUE));
  }

  public Command setColorOperationIdle() {
    return runOnce(() -> setColor(Color.ORANGE));
  }

  public Command setNoColorCommand() {
    return runOnce(() -> setColor(Color.OFF));
  }

  public FireAnimation burnyBurn() {
    return new FireAnimation(1, 0.75, NUM_LEDS, 1.0, 0.3);
  }

  public RainbowAnimation prettyLights() {
    return new RainbowAnimation(1, 1, NUM_LEDS);
  }

  public StrobeAnimation shoot() {
    return new StrobeAnimation(254, 254, 254, 254, 0.5, NUM_LEDS);
  }

  public ColorFlowAnimation prepareShoot() {
    return new ColorFlowAnimation(254, 55, 0, 0, 0.75, NUM_LEDS, Direction.Backward);
  }

  public RainbowAnimation rainbow() {
    return new RainbowAnimation();
  }

  public Command runBurnyBurnCommand() {
    return runOnce(() -> candle.animate(burnyBurn()));
  }

  public Command runPrettyLightsCommand() {
    return runOnce(() -> candle.animate(prettyLights()));
  }

  public Command runRainbowAnimationCommand() {
    return runOnce(() -> candle.animate(rainbow()));
  }

  public Command runPrepareShootCommand() {
    return runOnce(() -> candle.animate(prepareShoot()));
  }

  public Command runShootCommand() {
    return runOnce(() -> candle.animate(shoot()));
  }

  // TODO: Add following options: Blinking, quick-flash, fade/breathe mode, more colors???

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
