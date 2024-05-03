package frc.robot.subsystems.lineBreak;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.lineBreak.LineBreakHelper.LineBreakValues;

public class LineBreakIODigitalInput implements LineBreakIO {
  DigitalInput intakeSensor = new DigitalInput(7);

  public void updateInputs(LineBreakIOInputs inputs) {
    inputs.lineBreakValues = new LineBreakValues(!intakeSensor.get());
  }
}
