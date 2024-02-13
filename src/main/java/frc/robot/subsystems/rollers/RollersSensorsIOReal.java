package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import java.time.Duration;

public class RollersSensorsIOReal implements RollersSensorsIO {
  private final DigitalInput shooterStagedSensor = new DigitalInput(8);

  public RollersSensorsIOReal() {
    DigitalGlitchFilter glitchFilter = new DigitalGlitchFilter();
    glitchFilter.setPeriodNanoSeconds(Duration.ofMillis(5).toNanos());
    glitchFilter.add(shooterStagedSensor);
  }

  @Override
  public void updateInputs(RollersSensorsIOInputs inputs) {
    inputs.shooterStaged = shooterStagedSensor.get();
  }
}
