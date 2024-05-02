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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SmartController.DriveModeType;
import frc.robot.commands.AutoPreRoll;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.HandleLEDs;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualMagazine;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.SmartArm;
import frc.robot.commands.SmartIntake;
import frc.robot.commands.SmartMagazine;
import frc.robot.commands.SmartShoot;
import frc.robot.commands.SmartShooter;
import frc.robot.commands.VibrateController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIONeo;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.climber.ClimberLeft;
import frc.robot.subsystems.climber.ClimberRight;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIONeo;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeWheelsIO;
import frc.robot.subsystems.intake.IntakeWheelsIONeo;
import frc.robot.subsystems.intake.IntakeWheelsIOSim;
import frc.robot.subsystems.leds.LedController;
import frc.robot.subsystems.linebreak.LineBreak;
import frc.robot.subsystems.linebreak.LineBreakIO;
import frc.robot.subsystems.linebreak.LineBreakIODigitalInput;
import frc.robot.subsystems.linebreak.LineBreakIOSim;
import frc.robot.subsystems.magazine.Magazine;
import frc.robot.subsystems.magazine.MagazineIO;
import frc.robot.subsystems.magazine.MagazineIONeo;
import frc.robot.subsystems.magazine.MagazineIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVision;
import frc.robot.util.visualizer.NoteVisualizer;
import frc.robot.util.visualizer.RobotGamePieceVisualizer;
import frc.robot.util.visualizer.ShotVisualizer;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final AprilTagVision aprilTagVision;
  private final Arm arm;
  private final Intake intake;
  private final LineBreak lineBreak;
  private final Magazine magazine;

  private final LedController ledController;

  private boolean hasShot = false;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final ClimberLeft climberLeftPID = new ClimberLeft();
  private final ClimberRight climberRightPID = new ClimberRight();

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
        intake = new Intake(new IntakeWheelsIONeo());
        magazine = new Magazine(new MagazineIONeo());
        lineBreak = new LineBreak(new LineBreakIODigitalInput());
        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOPhotonVision("BLCamera", ROBOT_TO_CAMERA_BL),
                new AprilTagVisionIOPhotonVision("BRCamera", ROBOT_TO_CAMERA_BR),
                new AprilTagVisionIOPhotonVision("BackCamera", ROBOT_TO_CAMERA_BACK));
        ledController = new LedController(aprilTagVision);

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
        intake = new Intake(new IntakeWheelsIOSim());
        magazine = new Magazine(new MagazineIOSim());
        lineBreak = new LineBreak(new LineBreakIOSim());
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
        ledController = new LedController(aprilTagVision);
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
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
        arm = new Arm(new ArmIO() {});
        intake = new Intake(new IntakeWheelsIO() {});
        magazine = new Magazine(new MagazineIO() {});
        lineBreak = new LineBreak(new LineBreakIO() {});
        ledController = new LedController(aprilTagVision);
        break;
    }

    NoteVisualizer.setRobotPoseSupplier(
        () ->
            new Pose3d(
                new Translation3d(
                    drive.getPose().getTranslation().getX(),
                    drive.getPose().getTranslation().getY(),
                    0),
                new Rotation3d(0, 0, drive.getPose().getRotation().getRadians())));

    RobotGamePieceVisualizer.setRobotPoseSupplier(
        () ->
            new Pose3d(
                new Translation3d(
                    drive.getPose().getTranslation().getX(),
                    drive.getPose().getTranslation().getY(),
                    0),
                new Rotation3d(0, 0, drive.getPose().getRotation().getRadians())));

    RobotGamePieceVisualizer.setArmTransformSupplier(arm::getFlywheelPosition);
    RobotGamePieceVisualizer.setShooterAngleSupplier(arm::getArmAngleAbsolute);
    RobotGamePieceVisualizer.setIsMagazineLoadedSupplier(lineBreak::hasGamePieceIntake);
    RobotGamePieceVisualizer.setIsShooterLoadedSupplier(lineBreak::isShooterLoaded);

    NamedCommands.registerCommand(
        "Shoot",
        new SmartShoot(arm, shooter, magazine, lineBreak, drive::getPose, 1.5)
            .deadlineWith(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0))
            .andThen(
                new ScheduleCommand(
                    Commands.defer(() -> new ShotVisualizer(drive, arm, shooter), Set.of()))));
    NamedCommands.registerCommand(
        "QuickShoot",
        new SmartShoot(arm, shooter, magazine, lineBreak, drive::getPose, 1.0)
            .deadlineWith(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0))
            .andThen(
                new ScheduleCommand(
                    Commands.defer(() -> new ShotVisualizer(drive, arm, shooter), Set.of()))));

    NamedCommands.registerCommand(
        "EnableSmartControl", Commands.runOnce(SmartController.getInstance()::enableSmartControl));

    // Temporary workaround for the above line to prevent blocking at each pickup.
    // NamedCommands.registerCommand(
    // "SIMGamePiecePickup",
    // new ScheduleCommand(
    // Commands.defer(
    // () ->
    // new InstantCommand(
    // () -> lineBreak.setGamePiece(false, true, false, false, false,
    // false))
    // .andThen(Commands.waitSeconds(0.2))
    // .andThen(
    // new InstantCommand(
    // () ->
    // lineBreak.setGamePiece(
    // false, false, false, false, true, false))),
    // Set.of())));

    NamedCommands.registerCommand("SIMGamePiecePickup", Commands.none());

    NamedCommands.registerCommand(
        "SmartControl",
        Commands.parallel(
            new SmartShooter(shooter, lineBreak),
            new SmartArm(arm, lineBreak),
            new SmartIntake(intake, arm, lineBreak, magazine, arm::isArmInIntakePosition)));

    NamedCommands.registerCommand(
        "SmartIntake",
        new SmartIntake(intake, arm, lineBreak, magazine, arm::isArmInIntakePosition));

    NamedCommands.registerCommand(
        "PreRollShoot",
        Commands.deadline(
                new SmartShoot(arm, shooter, magazine, lineBreak, drive::getPose, 1.5),
                new SmartShooter(shooter, lineBreak),
                new SmartArm(arm, lineBreak),
                DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0))
            .andThen(
                Commands.runOnce(
                    () -> {
                      arm.setArmTarget(ArmConstants.intake.arm().getRadians());
                    })));

    NamedCommands.registerCommand(
        "PreRollShootAndMove",
        Commands.deadline(
                new SmartShoot(arm, shooter, magazine, lineBreak, drive::getPose, 0.5),
                new SmartShooter(shooter, lineBreak),
                new SmartArm(arm, lineBreak))
            .andThen(
                Commands.runOnce(
                    () -> {
                      arm.setArmTarget(ArmConstants.intake.arm().getRadians());
                    })));

    NamedCommands.registerCommand(
        "ManualUpCloseShot",
        new SequentialCommandGroup(
            new ManualShoot(arm, shooter, magazine, lineBreak, 0.5),
            Commands.runOnce(
                () -> {
                  arm.setArmTarget(ArmConstants.intake.arm().getRadians());
                })));

    NamedCommands.registerCommand(
        "PreRollShootFast",
        Commands.deadline(
            new SmartShoot(arm, shooter, magazine, lineBreak, drive::getPose, 0.5),
            new SmartShooter(shooter, lineBreak),
            new SmartArm(arm, lineBreak),
            DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0)));

    NamedCommands.registerCommand("IntakeDown", new InstantCommand(intake::enableIntakeRequest));
    NamedCommands.registerCommand("IntakeUp", new InstantCommand(intake::disableIntakeRequest));

    NamedCommands.registerCommand("Magazine", new ManualMagazine(magazine, lineBreak));

    NamedCommands.registerCommand(
        "Preload", new InstantCommand(() -> lineBreak.setGamePiece(false)));
    // PodiumShot
    // x=2.817 y=3.435
    NamedCommands.registerCommand(
        "PodiumPreroll", new AutoPreRoll(arm, shooter, lineBreak, ArmConstants.shoot.arm(), 2000));
    NamedCommands.registerCommand(
        "ClosePreroll", new AutoPreRoll(arm, shooter, lineBreak, ArmConstants.shoot.arm(), 2000));
    NamedCommands.registerCommand(
        "ManualPreroll",
        new AutoPreRoll(arm, shooter, lineBreak, ArmConstants.manualShot.arm(), 2000));
    // Run SmartController updates in autonomousma
    new Trigger(DriverStation::isAutonomousEnabled)
        .and(
            new Trigger(
                () -> SmartController.getInstance().getDriveModeType() == DriveModeType.SPEAKER))
        .whileTrue(
            new InstantCommand(
                () -> {
                  SmartController.getInstance()
                      .calculateSpeaker(
                          drive.getPose(), new Translation2d(0, 0), new Translation2d(0, 0));
                }));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    // Configure the button bindings
    aprilTagVision.setDataInterfaces(drive::addVisionData);
    SmartController.getInstance().disableSmartControl();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    arm.setDefaultCommand(new SmartArm(arm, lineBreak));
    shooter.setDefaultCommand(new SmartShooter(shooter, lineBreak));
    intake.setDefaultCommand(
        new SmartIntake(intake, arm, lineBreak, magazine, arm::isArmInIntakePosition)
            .ignoringDisable(true));
    magazine.setDefaultCommand(new SmartMagazine(magazine, intake, lineBreak));
    lineBreak.setDefaultCommand(
        new InstantCommand(RobotGamePieceVisualizer::drawGamePieces, lineBreak));
    ledController.setDefaultCommand(new HandleLEDs(ledController, lineBreak));
    // climber.setDefaultCommand(new SmartClimb(climber));

    driverController
        .start()
        .and(driverController.back())
        .whileTrue(
            Commands.runOnce(
                () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)));

    driverController
        .leftTrigger()
        .whileTrue(new SmartShoot(arm, shooter, magazine, lineBreak, drive::getPose, 1.5));

    driverController
        .rightTrigger()
        .whileTrue(
            Commands.startEnd(intake::enableIntakeRequest, intake::disableIntakeRequest)
                .deadlineWith(new VibrateController(driverController, lineBreak)));

    driverController
        .a()
        .whileTrue(Commands.runOnce(SmartController.getInstance()::enableSmartControl));

    driverController
        .b()
        .whileTrue(Commands.runOnce(SmartController.getInstance()::disableSmartControl));

    driverController
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                Commands.run(() -> SmartController.getInstance().setDriveMode(DriveModeType.AMP)),
                Commands.run(intake::outtake, intake),
                Commands.run(magazine::backward, magazine)))
        .onFalse(
            Commands.runOnce(
                () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)));

    driverController
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                Commands.run(() -> SmartController.getInstance().setDriveMode(DriveModeType.AMP)),
                Commands.run(intake::intake, intake),
                Commands.run(magazine::forward, magazine),
                Commands.run(() -> shooter.setSetpoint(600, 600), shooter)))
        .onFalse(
            Commands.runOnce(
                () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)));

    driverController
        .y()
        .whileTrue(new ManualIntake(arm, shooter, ArmConstants.intake, () -> -5, lineBreak));

    operatorController
        .a()
        .whileTrue(
            Commands.run(() -> SmartController.getInstance().setDriveMode(DriveModeType.AMP)))
        .onFalse(
            Commands.runOnce(
                () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)));

    operatorController
        .x()
        .whileTrue(
            Commands.startEnd(
                () -> SmartController.getInstance().setDriveMode(DriveModeType.FEED),
                () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)));

    // operatorController
    // .leftTrigger(0.5)
    // .and(operatorController.b())
    // .onTrue(new ManualClimber(climber, 5.2, 0))
    // .onFalse(new ManualClimber(climber, 2.4, 1));

    // operatorController
    // .pov(180)
    // .toggleOnTrue(
    // Commands.startEnd(
    // () -> climber.setRequestingClimb(true),
    // () -> climber.setRequestingClimb(false)));
    operatorController
        .pov(0)
        .toggleOnTrue(
            Commands.either(
                Commands.runEnd(
                    () -> SmartController.getInstance().setDriveMode(DriveModeType.CLIMBER),
                    () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)),
                Commands.runEnd(
                    () -> SmartController.getInstance().setDriveMode(DriveModeType.QUICK_CLIMB),
                    () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)),
                lineBreak::hasGamePiece));

    driverController.x().whileTrue(new ManualShoot(arm, shooter, magazine, lineBreak, 1.5));

    if (Constants.getMode() == Constants.Mode.SIM) {
      driverController.pov(180).onTrue(new InstantCommand(lineBreak::shootGamePiece));
      driverController.pov(90).onTrue(new InstantCommand(() -> lineBreak.setGamePiece(true)));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void intakeUp() {
    intake.disableIntakeRequest();
  }
}
