// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PositionClimbLeftPID;
import frc.robot.commands.PositionClimbRightPID;
import frc.robot.commands.VelocityShootCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.ClimberLeft;
import frc.robot.subsystems.climber.ClimberRight;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIO;
import frc.robot.subsystems.rollers.intake.IntakeIOKrakenFOC;
import frc.robot.subsystems.rollers.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVisionSIM;
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
  private static DriveController driveMode = new DriveController();
  private AprilTagVision aprilTagVision;
  private final Shooter shooter = new Shooter();
  private final Rollers rollers;
  private final ClimberLeft climberleft;
  private final ClimberRight climberright;

  private final Arm m_arm = new Arm();

  private boolean hasRun = false;
  private boolean hasEjected = false; // New flag for the EJECTALIGN command

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // private final LoggedTunableNumber flywheelSpeedInput =
  // new LoggedTunableNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Declare component subsystems (not visible outside constructor)
    Intake intake = null;
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(moduleConfigs[0]),
                new ModuleIOTalonFX(moduleConfigs[1]),
                new ModuleIOTalonFX(moduleConfigs[2]),
                new ModuleIOTalonFX(moduleConfigs[3]));
        aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight("limelight-back"));
        intake = new Intake(new IntakeIOKrakenFOC());
        rollers = new Rollers(intake);
        // arm = new Arm(new ArmIOKrakenFOC());
        climberleft = new ClimberLeft();
        climberright = new ClimberRight();

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
        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOPhotonVisionSIM(
                    "photonCamera1",
                    new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)),
                    drive::getDrive));
        intake = new Intake(new IntakeIOSim());
        rollers = new Rollers(intake);

        // arm = new Arm(new ArmIOSim());
        climberleft = new ClimberLeft();
        climberright = new ClimberRight();

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

        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});

        intake = new Intake(new IntakeIO() {});

        // arm = new Arm(new ArmIO() {});

        rollers = new Rollers(intake);

        climberleft = new ClimberLeft();
        climberright = new ClimberRight();
    }

    // Set up auto routines
    // NamedCommands.registerCommand(
    // "Run Flywheel",
    // Commands.startEnd(
    // () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
    // flywheel)
    // .withTimeout(5.0));

    // ================================================
    // Register the Auto Command Intake
    // ================================================
    NamedCommands.registerCommand(
        "Intake",
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (!hasRun) {
                    rollers.setGoal(Rollers.Goal.FLOOR_INTAKE);
                    hasRun = true;
                  }
                },
                rollers),
            Commands.waitUntil(() -> rollers.getBeamBreak()),
            Commands.runOnce(
                () -> {
                  if (!hasEjected) {
                    rollers.setGoal(Rollers.Goal.EJECTALIGN);
                    hasEjected = true;
                  }
                },
                rollers),
            Commands.waitUntil(() -> !rollers.getBeamBreak()),
            Commands.runOnce(
                () -> {
                  rollers.setGoal(Rollers.Goal.IDLE);
                })));

    // ================================================
    // Register the Auto Command Intake Reset
    // ================================================
    NamedCommands.registerCommand(
        "Intake Reset",
        Commands.runOnce(
            () -> {
              hasRun = false;
              hasEjected = false;
            }));

    // ================================================
    // Register the Auto Command PreShoot
    // ================================================
    NamedCommands.registerCommand(
        "PreShoot", Commands.runOnce(() -> shooter.setVelocity(30.0, 15.0)));

    // ================================================
    // Register the Auto Command Shoot
    // ================================================
    NamedCommands.registerCommand(
        "Shoot",
        Commands.sequence(
            Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.FEED_SHOOTER)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  shooter.setVelocity(0);
                  rollers.setGoal(Rollers.Goal.IDLE);
                })));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Flywheel SysId (Quasistatic Forward)",
    // flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Flywheel SysId (Quasistatic Reverse)",
    // flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Flywheel SysId (Dynamic Forward)",
    // flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Flywheel SysId (Dynamic Reverse)",
    // flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    aprilTagVision.setDataInterfaces(drive::addVisionData);
    driveMode.setPoseSupplier(drive::getPose);
    driveMode.disableHeadingControl();
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
            driveMode,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> driverController.getRightX()));

    driverController
        .x()
        .whileTrue( // Tyler Fixed This. :)
            Commands.sequence(
                Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.FLOOR_INTAKE), rollers),
                Commands.waitUntil(() -> rollers.getBeamBreak()),
                Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.EJECTALIGN)),
                Commands.waitUntil(() -> !rollers.getBeamBreak()),
                Commands.waitSeconds(0.035),
                Commands.runOnce(
                    () -> {
                      rollers.setGoal(Rollers.Goal.IDLE);
                    })))
        .onFalse(
            Commands.runOnce(
                () -> {
                  rollers.setGoal(Rollers.Goal.IDLE);
                }));

    driverController
        .a()
        .whileTrue( // Tyler Fixed This. :)
            Commands.sequence(
                Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.EJECT_TO_FLOOR), rollers)))
        .whileFalse(
            Commands.runOnce(
                () -> {
                  rollers.setGoal(Rollers.Goal.IDLE);
                }));

    driverController
        .start()
        .whileTrue(
            Commands.runOnce(
                () ->
                    drive.setAutoStartPose(
                        new Pose2d(new Translation2d(4, 5), Rotation2d.fromDegrees(0)))));

    driverController.y().whileTrue(new VelocityShootCommand(shooter));

    // driverController
    // .povDown()
    // .whileTrue(
    // new DriveToPoint(
    // drive, new Pose2d(new Translation2d(2.954, 3.621),
    // Rotation2d.fromRadians(2.617))));

    // driverController
    // .povUp()
    // .whileTrue(
    // new MultiDistanceShot(
    // drive::getPose,
    // FieldConstants.Speaker.centerSpeakerOpening.getTranslation(),
    // flywheel));

    driverController.leftBumper().whileTrue(new PositionClimbLeftPID(climberleft, 300));
    driverController.leftTrigger().whileTrue(new PositionClimbLeftPID(climberleft, -300));
    driverController.rightBumper().whileTrue(new PositionClimbRightPID(climberright, 300));
    driverController.rightTrigger().whileTrue(new PositionClimbRightPID(climberright, -300));

    driverController
        .povUp()
        .whileTrue(
            new PositionClimbLeftPID(climberleft, 300)
                .alongWith(new PositionClimbRightPID(climberright, 300)));
    driverController
        .povDown()
        .whileTrue(
            new PositionClimbLeftPID(climberleft, -300)
                .alongWith(new PositionClimbRightPID(climberright, -300)));

    operatorController.povUp().onTrue(m_arm.armUp());
    operatorController.povDown().onTrue(m_arm.armDown());
    operatorController.povRight().onTrue(m_arm.armUpMicro());
    operatorController.povLeft().onTrue(m_arm.armDownMicro());

    operatorController.rightBumper().whileTrue(shooter.differentialShootDownCommand()); // subwoofer
    operatorController.leftBumper().whileTrue(shooter.differentialShootUpCommand()); // midrange
    operatorController.leftTrigger().whileTrue(shooter.commonShootCommand()); // normal
    operatorController.rightTrigger().whileTrue(shooter.farShootCommand()); // far

    operatorController.x().onTrue(m_arm.armBackZero());
    operatorController.y().onTrue(m_arm.armAMP());
    operatorController.a().onTrue(m_arm.armMid());
    operatorController.b().onTrue(m_arm.armPod());

    // driverController.povUp().whileTrue(Commands.runOnce(() ->
    // arm.setDesiredDegrees(90)));

    // controller
    // .b()
    // .onTrue(
    // Commands.runOnce(
    // () ->
    // drive.setPose(
    // new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    // drive)
    // .ignoringDisable(true));
    // controller
    // .a()
    // .whileTrue(
    // Commands.startEnd(
    // () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
    // flywheel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  /** Creates a controller rumble command with specified rumble and controllers */
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
