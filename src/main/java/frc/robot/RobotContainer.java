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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.PathFinderAndFollow;
import frc.robot.commands.SmartShoot;
import frc.robot.commands.SmartShooter;
import frc.robot.commands.arm.PositionArmPID;
import frc.robot.commands.arm.SmartArm;
import frc.robot.commands.climber.ManualClimb;
import frc.robot.subsystems.arm.Arm;
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
import frc.robot.subsystems.leds.Candle;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.Goal;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.rollers.feeder.FeederIO;
import frc.robot.subsystems.rollers.feeder.FeederIOSim;
import frc.robot.subsystems.rollers.feeder.FeederIOSparkFlexBack;
import frc.robot.subsystems.rollers.feeder.FeederIOSparkFlexFront;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIO;
import frc.robot.subsystems.rollers.intake.IntakeIOSim;
import frc.robot.subsystems.rollers.intake.IntakeIOSparkFlex;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVision;
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
  private Feeder feeder1;
  private Feeder feeder2;
  private Rollers rollers;
  private Candle candle = new Candle();

  private boolean hasShot = false;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

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
        feeder1 = new Feeder(new FeederIOSparkFlexFront());
        feeder2 = new Feeder(new FeederIOSparkFlexBack());
        intake = new Intake(new IntakeIOSparkFlex());
        rollers = new Rollers(feeder1, feeder2, intake);

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
        feeder1 = new Feeder(new FeederIOSim());
        feeder2 = new Feeder(new FeederIOSim());
        intake = new Intake(new IntakeIOSim());
        rollers = new Rollers(feeder1, feeder2, intake);

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

        feeder1 = new Feeder(new FeederIO() {});

        feeder2 = new Feeder(new FeederIO() {});

        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});

        intake = new Intake(new IntakeIO() {});
    }
    // ================================================
    // Register the Named Commands
    // ================================================
    NamedCommands.registerCommand("Intake", intakeNote());
    NamedCommands.registerCommand("Eject", ejectNote());
    NamedCommands.registerCommand("blurpShoot", blurpShoot());

    // ================================================
    // Register the Auto Command AimAndPreShoot
    // ================================================
    NamedCommands.registerCommand("AimAndPreShoot", aimAndPreShoot());

    // ================================================
    // Register the Auto Command BlurpShoot
    // ================================================
    NamedCommands.registerCommand("BlurpShoot", blurpShoot());

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

  public Command intakeNote() {
    return Commands.sequence(
        candle.runPrettyLightsCommand(),
        Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.FLOOR_INTAKE), rollers),
        Commands.waitUntil(() -> !rollers.getBeamBreak()),
        Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.EJECTALIGN)),
        candle.setColorGreenCommand(),
        Commands.waitUntil(() -> rollers.getBeamBreak()),
        Commands.runOnce(
            () -> {
              rollers.setGoal(Rollers.Goal.IDLE);
            }),
        Commands.waitSeconds(0.1),
        candle.setColorRespawnIdle());
  }

  public Command ejectNote() {
    return Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.EJECT_TO_FLOOR), rollers);
  }

  public Command aimAndPreShoot() {
    return Commands.sequence(
            Commands.run(() -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)))
        .until(() -> hasShot)
        .andThen(
            Commands.runOnce(() -> SmartController.getInstance().setDriveMode(DriveModeType.SAFE)));
  }

  public Command blurpShoot() {
    return Commands.sequence(
            Commands.startEnd(
                () -> shooter.setSetpoint(3000, 3000), () -> shooter.setSetpoint(0, 0)))
        .until(() -> hasShot);
  }

  public Command autoBlurp() {
    return Commands.sequence(
        // Start by spinning up the shooter and setting the arm to 40
        Commands.parallel(
            Commands.runOnce(() -> shooter.setSetpoint(4000, 4000)),
            Commands.runOnce(() -> new PositionArmPID(arm, 40))));
  }

  public void resetHasShot() {
    hasShot = false;
  }

  public Command shoot() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              rollers.setGoal(Rollers.Goal.SHOOT);
              candle.runShootCommand();
            }),
        Commands.waitSeconds(0.4),
        Commands.runOnce(
            () -> {
              hasShot = true; // set hasShot to true
              rollers.setGoal(Rollers.Goal.IDLE);
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

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
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

    // ================================================
    // DRIVER CONTROLLER - LEFT BUMPER
    // RUN INTAKE IN
    // ================================================
    driverController
        .leftBumper()
        .whileTrue(intakeNote())
        .onFalse(
            Commands.runOnce(
                () -> {
                  rollers.setGoal(Rollers.Goal.IDLE);
                }));

    // ================================================
    // DRIVER CONTROLLER - LEFT TRIGGER
    // RUN INTAKE OUT
    // ================================================
    driverController
        .leftTrigger()
        .whileTrue(ejectNote())
        .onFalse(
            Commands.runOnce(
                () -> {
                  rollers.setGoal(Rollers.Goal.IDLE);
                }));

    // ================================================
    // DRIVER CONTROLLER - RIGHT BUMPER
    // SET DRIVE MODE TO SPEAKER
    // ================================================
    operatorController
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)));

    // ================================================
    // DRIVER CONTROLLER - DPAD DOWN
    // MOVE CLIMBER DOWN
    // ================================================
    driverController
        .rightTrigger()
        .whileTrue(new ManualClimb(climberLeft, climberRight, -140))
        .whileFalse(new ManualClimb(climberLeft, climberRight, 0));

    // ================================================
    // DRIVER CONTROLLER - A
    // PATHFIND TO AMP
    // ================================================
    driverController.a().whileTrue(new PathFinderAndFollow("Amp Placement Path"));

    // ================================================
    // DRIVER CONTROLLER - B
    // PATHFIND TO STAGE SHOOT
    // ================================================
    driverController
        .b()
        .whileTrue(
            new DriveToPoint(
                drive, new Pose2d(new Translation2d(3.9, 5.0), Rotation2d.fromDegrees(345))));

    // ========================================================================================================
    // ========================================================================================================

    // ================================================
    // OPERATOR CONTROLLER - LB
    // SCORE AMP
    // ================================================
    operatorController
        .leftBumper()
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.AMP_SHOOTER), rollers)))
        .onFalse(
            Commands.runOnce(
                () -> {
                  rollers.setGoal(Rollers.Goal.IDLE);
                }));

    // ================================================
    // OPERATOR CONTROLLER - RB
    // SCORE AMP RED
    // ================================================
    operatorController
        .rightBumper()
        .whileTrue(Commands.runOnce(() -> rollers.setGoal(Goal.SHOOT), rollers))
        .onFalse(Commands.runOnce(() -> rollers.setGoal(Goal.IDLE), rollers));

    // ================================================
    // OPERATOR CONTROLLER - RIGHT TRIGGER
    // SHOOT
    // ================================================
    operatorController
        .rightTrigger()
        .whileTrue(new SmartShoot(arm, shooter, rollers, drive::getPose, 1));

    // ================================================
    // OPERATOR CONTROLLER - A
    // SETS SHOOTER TO BLURP SHOOT SPEED
    // ================================================
    operatorController
        .a()
        .onTrue(
            Commands.runOnce(() -> SmartController.getInstance().setDriveMode(DriveModeType.FEED))
                .alongWith(
                    Commands.runOnce(() -> SmartController.getInstance().enableSmartControl())))
        .onFalse(Commands.runOnce(() -> SmartController.getInstance().disableSmartControl()));

    // ================================================
    // OPERATOR CONTROLLER - DPAD LEFT
    // ARM POSITION AMP
    // ================================================
    operatorController
        .povLeft()
        .whileTrue(
            Commands.runOnce(() -> SmartController.getInstance().setDriveMode(DriveModeType.AMP))
                .alongWith(
                    Commands.runOnce(() -> SmartController.getInstance().enableSmartControl())))
        .onFalse(Commands.runOnce(() -> SmartController.getInstance().disableSmartControl()));

    // ================================================
    // OPERATOR CONTROLLER - DPAD DOWN
    // ARM POSITION LOWEST POSITION
    // ================================================
    operatorController
        .povDown()
        .onTrue(
            Commands.runOnce(() -> SmartController.getInstance().setDriveMode(DriveModeType.SAFE)));
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
