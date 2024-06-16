// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.vision.CameraConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.beambreak.*;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.leds.Candle;
import frc.robot.subsystems.magazine.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private Drive drive;
  private Shooter shooter;
  private AprilTagVision aprilTagVision;
  private Arm arm;
  private Intake intake;
  private BeamBreak beamBreak;
  private Magazine magazine;
  private Candle candle = new Candle();

  private boolean hasShot = false;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final ClimberLeft climberLeft = new ClimberLeft();
  private final ClimberRight climberRight = new ClimberRight();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware I` implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIONeo(moduleConfigs[0]),
                new ModuleIONeo(moduleConfigs[1]),
                new ModuleIONeo(moduleConfigs[2]),
                new ModuleIONeo(moduleConfigs[3]));

        shooter = new Shooter(new ShooterIOSparkFlex());
        arm = new Arm(new ArmIONeo());
        magazine = new Magazine(new MagazineIONeo());
        beamBreak = new BeamBreak(new BeamBreakIODigitalInput());
        intake = new Intake(new IntakeIONeo());

        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOPhotonVision("BLCamera", ROBOT_TO_CAMERA_BL),
                new AprilTagVisionIOPhotonVision("BRCamera", ROBOT_TO_CAMERA_BR),
                new AprilTagVisionIOPhotonVision("BackCamera", ROBOT_TO_CAMERA_BACK));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        shooter = new Shooter(new ShooterIOSim());
        arm = new Arm(new ArmIOSim());
        magazine = new Magazine(new MagazineIOSim());
        beamBreak = new BeamBreak(new BeamBreakIOSim());
        intake = new Intake(new IntakeIOSim());

        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter = new Shooter(new ShooterIO() {});

        arm = new Arm(new ArmIO() {});

        magazine = new Magazine(new MagazineIO() {});

        beamBreak = new BeamBreak(new BeamBreakIO() {});

        intake = new Intake(new IntakeIO() {});

        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
    }
    // ================================================
    // Register the Named Commands
    // ================================================
    NamedCommands.registerCommand("Intake", new InstantCommand(intake::enableIntakeRequest));

    // ================================================
    // Register the Auto Command AimAndPreShoot
    // ================================================
    NamedCommands.registerCommand("AimAndPreShoot", aimAndPreShoot());

    // ================================================
    // Register the Auto Command Shoot
    // ================================================
    NamedCommands.registerCommand("Shoot", shoot());

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    aprilTagVision.setDataInterfaces(drive::addVisionData);
    SmartController.getInstance().disableSmartControl();
    configureButtonBindings();
  }

  public Command aimAndPreShoot() {
    return Commands.runOnce(
        () -> new SmartShoot(arm, shooter, magazine, beamBreak, drive::getPose, 1));
  }

  public Command manualBlurp() {
    return Commands.sequence(
        Commands.runOnce(() -> shooter.manualBlurp()),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> magazine.shoot()),
        Commands.runOnce(() -> shooter.setSetpoint(0, 0)));
  }

  public void resetHasShot() {
    hasShot = false;
  }

  public Command shoot() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              magazine.shoot();
              candle.runShootCommand();
            }),
        Commands.waitSeconds(0.4),
        Commands.runOnce(
            () -> {
              hasShot = true; // set hasShot to true
              // rollers.setGoal(Rollers.Goal.IDLE);
              shooter.stop();
              candle.setColorRespawnIdle();
            }),
        Commands.runOnce(() -> resetHasShot())); // reset hasShot);
  }

  public Command resetHeading() {
    return Commands.runOnce(
            () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
            drive)
        .ignoringDisable(true);
  }

  private void configureButtonBindings() {

    // ==================
    // DEFAULT COMMANDS
    // ==================
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    arm.setDefaultCommand(new SmartArm(arm));
    shooter.setDefaultCommand(new SmartShooter(shooter));
    intake.setDefaultCommand(
        new SmartIntake(intake, beamBreak, magazine, candle).ignoringDisable(true));
    magazine.setDefaultCommand(new SmartMagazine(magazine, intake, beamBreak, candle));

    // ================================================
    // DRIVER CONTROLLER - LEFT BUMPER
    // RUN INTAKE IN
    // ================================================
    driverController
        .leftBumper()
        .whileTrue(Commands.startEnd(intake::enableIntakeRequest, intake::disableIntakeRequest));

    // ================================================
    // DRIVER CONTROLLER - LEFT TRIGGER
    // RUN INTAKE OUT
    // ================================================
    driverController
        .leftTrigger()
        .whileTrue(Commands.startEnd(intake::enableOuttakeRequest, intake::disableOuttakeRequest));

    // .onFalse(
    // Commands.runOnce(
    // () -> {
    // rollers.setGoal(Rollers.Goal.IDLE);
    // }));

    // ================================================
    // DRIVER CONTROLLER - A
    // SET DRIVE MODE TO AMP
    // ================================================
    driverController
        .a()
        .onTrue(
            Commands.runOnce(() -> SmartController.getInstance().setDriveMode(DriveModeType.AMP))
                .alongWith(
                    Commands.runOnce(() -> SmartController.getInstance().enableSmartControl())));

    // ================================================
    // DRIVER CONTROLLER - B
    // SET DRIVE MODE TO SPEAKER
    // ================================================
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER))
                .alongWith(
                    Commands.runOnce(() -> SmartController.getInstance().enableSmartControl())));

    // ================================================
    // DRIVER CONTROLLER - X
    // SET DRIVE MODE TO SAFE
    // ================================================
    driverController
        .x()
        .onTrue(
            Commands.runOnce(() -> SmartController.getInstance().setDriveMode(DriveModeType.SAFE)));

    // ================================================
    // DRIVER CONTROLLER - Y
    // SET DRIVE MODE TO FEED
    // ================================================
    driverController
        .y()
        .onTrue(
            Commands.runOnce(() -> SmartController.getInstance().setDriveMode(DriveModeType.FEED))
                .alongWith(
                    Commands.runOnce(() -> SmartController.getInstance().enableSmartControl())));

    // ================================================
    // DRIVER CONTROLLER - RIGHT TRIGGER
    // SCORE
    // ================================================
    driverController
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () -> {
                  if (SmartController.getInstance().getDriveModeType() == DriveModeType.AMP) {
                    Commands.runOnce(() -> magazine.outtake());
                  } else if (SmartController.getInstance().getDriveModeType()
                      == DriveModeType.SPEAKER) {
                    new SmartShoot(arm, shooter, magazine, beamBreak, drive::getPose, 1);
                  } else if (SmartController.getInstance().getDriveModeType()
                      == DriveModeType.FEED) {
                    Commands.runOnce(() -> magazine.shoot());
                  }
                }));

    // ================================================
    // DRIVER CONTROLLER - DPAD UP
    // MOVE CLIMBER UP
    // ================================================
    driverController.povUp().onTrue(new ManualClimb(climberLeft, climberRight, -140));

    // ================================================
    // DRIVER CONTROLLER - DPAD DOWN
    // MOVE CLIMBER DOWN
    // ================================================
    driverController.povDown().onTrue(new ManualClimb(climberLeft, climberRight, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
