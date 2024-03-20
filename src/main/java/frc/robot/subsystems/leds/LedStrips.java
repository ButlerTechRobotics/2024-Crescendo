// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

// import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AddressableLED; // LED模块
import edu.wpi.first.wpilibj.AddressableLEDBuffer; // LED显存
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants;

/**
 * LedStrips Subsystem :Used to turn Led Strips On Note: This Subsystem is writen in Singleton mode
 * use getIns() to get the instence
 */
public class LedStrips extends SubsystemBase {
  private final int LED_PORT = 0;
  private final int LED_LENGTH = 120;

  // Constant for light mode setting

  /** 当前不可用 Light Mode SINGLE: 单色设置，默认模式 SCROLLER:跑马灯模式 RAINBOW: 彩虹模式 */
  enum MODE {
    /** SINGLE: 单色模式 */
    SINGLE,
    /** SCROLLER: 跑马灯模式 */
    SCROLLER,
    /** RAINBOW: 彩虹模式 */
    RAINBOW;
  }

  private static MODE m_mode = MODE.SINGLE; // 当前不可用
  // LED Object and buffer object for leds strip, Port 0 as default.
  private AddressableLED m_LEDsStrip = new AddressableLED(LED_PORT);
  private AddressableLEDBuffer m_LEDsBuffer = new AddressableLEDBuffer(LED_LENGTH);

  // Timer for light effect
  Timer m_t = new Timer();

  public LedStrips() {
    m_LEDsStrip.setLength(m_LEDsBuffer.getLength());

    setRGB(255, 0, 0);
  }

  public Command setRGB_CMD(int red, int green, int blue) {
    return runOnce(() -> setRGB(red, green, blue));
  }

  public Command setRGB_CMD(int red, int green, int blue, int... index) {
    return runOnce(() -> setRGB(red, green, blue, index));
  }

  // Methods which support the function above

  /**
   * Turn All light to the given color
   *
   * @param red
   * @param green
   * @param blue
   */
  private void setRGB(int red, int green, int blue) {
    m_mode = MODE.SINGLE;
    for (int i = 0; i < this.m_LEDsBuffer.getLength(); i++) {
      this.m_LEDsBuffer.setRGB(i, red, green, blue);
    }
    this.m_LEDsStrip.setData(m_LEDsBuffer); // may could be deleted
    this.m_LEDsStrip.start();
  }

  /**
   * Change the specific LED color to the given color
   *
   * @param index which light to operate on, start from 0
   * @param red
   * @param green
   * @param blue
   */
  private void setRGB(int red, int green, int blue, int... index) {
    m_mode = MODE.SINGLE;
    for (int i : index) this.m_LEDsBuffer.setRGB(i, red, green, blue);
    this.m_LEDsStrip.setData(m_LEDsBuffer); // may could be deleted
    this.m_LEDsStrip.start();
  }
} // end of the whole class
