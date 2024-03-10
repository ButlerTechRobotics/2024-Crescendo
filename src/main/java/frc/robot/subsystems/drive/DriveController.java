package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.Optional;
import java.util.function.Supplier;

// DriveController class handles the driving logic for the robot
public class DriveController {
  // Supplier for the heading angle of the robot
  private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();
  // Supplier for the pose of the robot
  private Supplier<Pose2d> poseSupplier = Pose2d::new;
  // Current drive mode of the robot
  private DriveModeType driveModeType = DriveModeType.SPEAKER;

  // Sets the supplier for the heading angle of the robot
  public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = Optional.of(headingSupplier);
  }

  // Sets the supplier for the pose of the robot
  public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  // Checks if the heading of the robot is controlled
  public boolean isHeadingControlled() {
    return this.headingSupplier.isPresent();
  }

  // Returns the current drive mode of the robot
  public Supplier<DriveModeType> getDriveModeType() {
    return () -> this.driveModeType;
  }

  // Returns the current heading angle of the robot
  public Supplier<Rotation2d> getHeadingAngle() {
    return headingSupplier.get();
  }

  // Sets the drive mode of the robot
  public void setDriveMode(DriveModeType driveModeType) {
    this.driveModeType = driveModeType;
    updateHeading();
  }

  // Enables heading control for the robot
  public void enableHeadingControl() {
    enableSpeakerHeading();
  }

  // Disables heading control for the robot
  public void disableHeadingControl() {
    this.headingSupplier = Optional.empty();
  }

  // Updates the heading of the robot based on the current drive mode
  private void updateHeading() {
    if (isHeadingControlled()) {
      enableHeadingControl();
    }
  }

  // Enables speaker heading for the robot
  private void enableSpeakerHeading() {
    setHeadingSupplier(
        () ->
            new Rotation2d(
                poseSupplier.get().getX()
                    - AllianceFlipUtil.apply(
                            FieldConstants.Speaker.centerSpeakerOpening.getTranslation())
                        .getX(),
                poseSupplier.get().getY()
                    - AllianceFlipUtil.apply(
                            FieldConstants.Speaker.centerSpeakerOpening.getTranslation())
                        .getY()));
  }

  // Enum for the drive modes of the robot
  public enum DriveModeType {
    SPEAKER,
  }
}