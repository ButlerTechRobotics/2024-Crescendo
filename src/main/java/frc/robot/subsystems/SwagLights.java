package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.superstructure.Superstructure;

public class SwagLights implements Subsystem {
  // Enums
  public enum RobotStates {
    Enabled("A"),
    Disabled("D"),
    EmergencyStop("E");

    public final String value;

    private RobotStates(String value) {
      this.value = value;
    }
  }

  public enum LedStates {
    Default("D"),
    NoteIntakeRunning("N"),
    NoteIntakeIntaked("I"),
    Aiming("A"),
    Pathfinding("G"),
    PrepareShoot("P");

    public final String value;

    private LedStates(String value) {
      this.value = value;
    }
  }

  // Inputs and Outputs
  private SerialPort serialPort;

  // States
  private RobotStates robotState = RobotStates.Disabled;
  private LedStates ledState = LedStates.Default;

  // Singleton Setup
  private static SwagLights instance;

  public static SwagLights getInstance() {
    if (instance == null) {
      instance = new SwagLights();
    }
    return instance;
  }

  /** Initializes the serial communication */
  private SwagLights() {
    serialPort = new SerialPort(9600, SerialPort.Port.kMXP);
    serialPort.enableTermination();
    System.out.println("Created new serial reader");
  }

  @Override
  public void periodic() {
    if (this.robotState == RobotStates.Enabled) {
      setLedStates(this.robotState.value, this.ledState.value);
    } else {
      setLedStates(this.robotState.value);
    }
  }

  /**
   * Write an array of strings to the serial bus as a single sting
   *
   * @param values
   */
  private void setLedStates(String... values) {
    serialPort.writeString(String.join("", values));
  }

  /** Closes serial port listener */
  public void stopSerialPort() {
    System.out.println("Closing serial port");
    serialPort.close();
  }

  /**
   * Tells the swag lights the robot is disabled
   *
   * @param isDisabled
   */
  public void setDisabled(boolean isDisabled) {
    this.robotState = isDisabled ? RobotStates.Disabled : RobotStates.Enabled;
  }

  /** Tells the swag lights the robot is e-stopped */
  public void setEStop() {
    this.robotState = RobotStates.EmergencyStop;
  }

  /**
   * Tells the swag lights which operator mode we are in (cone or cube)
   *
   * @param isCube is the current mode the cube mode?
   */
  public void setLedState(Rollers.Goal ledState) {
    switch (ledState) {
      case FLOOR_INTAKE:
        this.ledState = LedStates.NoteIntakeRunning;
        break;
      case EJECTALIGN:
        this.ledState = LedStates.NoteIntakeIntaked;
        break;
    }
  }

  public void setLedState(Superstructure ledState) {
    switch (ledState.getGoal()) {
      case PREPARE_SHOOT:
        this.ledState = LedStates.PrepareShoot;
        break;
    }
  }
}
