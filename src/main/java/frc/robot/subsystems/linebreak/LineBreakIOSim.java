package frc.robot.subsystems.lineBreak;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.lineBreak.LineBreakHelper.LineBreakValues;

public class LineBreakIOSim implements LineBreakIO {
  NetworkTable table;
  NetworkTableEntry intakeSensor;

  public LineBreakIOSim() {
    table = NetworkTableInstance.getDefault().getTable("LineBreak");
    intakeSensor = table.getEntry("intakeSensor");
    intakeSensor.setBoolean(false);
  }

  @Override
  /** Bumps the game piece to the next sensor. */
  public void bumpGamePiece() {
    if (intakeSensor.getBoolean(false)) {
      intakeSensor.setBoolean(false);
    }
  }

  @Override
  /** Shoots the game piece (empty magazines) */
  public void shootGamePiece() {
    intakeSensor.setBoolean(false);
  }

  @Override
  /** Sets the game piece sensors */
  public void setGamePiece(boolean intake) {
    intakeSensor.setBoolean(intake);
  }

  public void updateInputs(LineBreakIOInputs inputs) {
    inputs.lineBreakValues = new LineBreakValues(intakeSensor.getBoolean(false));
  }
}
