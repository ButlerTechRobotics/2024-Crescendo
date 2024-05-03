package frc.robot.subsystems.lineBreak;

public class LineBreakHelper {
  public record LineBreakValues(boolean intake) {
    // Gets the value of the digital input. Returns true if the circuit is open.
    // True = game piece

    public boolean inIntake() {
      return intake;
    }

    public boolean hasGamePiece() {
      return intake;
    }

    public boolean notInIntake() {
      return !(intake);
    }

    public boolean hasGamePieceIntake() {
      return intake;
    }

    public boolean isShooterLoaded() {
      return (hasGamePieceIntake());
    }
  }
}
