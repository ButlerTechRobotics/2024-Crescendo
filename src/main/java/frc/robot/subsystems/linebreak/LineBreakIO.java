package frc.robot.subsystems.lineBreak;

import frc.robot.subsystems.lineBreak.LineBreakHelper.LineBreakValues;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LineBreakIO {
  class LineBreakIOInputs implements LoggableInputs {

    LineBreakValues lineBreakValues = new LineBreakValues(false);

    @Override
    public void toLog(LogTable table) {
      table.put("LineBreaker/intake", lineBreakValues.intake());
    }

    @Override
    public void fromLog(LogTable table) {
      table.get("LineBreaker/intake", lineBreakValues.intake());
    }
  }

  public default void bumpGamePiece() {}

  public default void shootGamePiece() {}

  public default void setGamePiece(boolean intake) {}

  default void updateInputs(LineBreakIOInputs inputs) {}
}
