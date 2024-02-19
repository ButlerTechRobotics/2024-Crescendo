// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MultiDistanceArm;
import frc.robot.commands.PathFinderAndFollow;
// import frc.robot.commands.ShootDistance;
import frc.robot.commands.arm.PositionArmPID;
// import frc.robot.commands.climber.PositionClimbPID;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.drive.DriveController.DriveModeType;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.rollers.feeder.FeederIO;
import frc.robot.subsystems.rollers.feeder.FeederIOSparkFlexBack;
import frc.robot.subsystems.rollers.feeder.FeederIOSparkFlexFront;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIO;
import frc.robot.subsystems.rollers.intake.IntakeIOSparkFlex;
// import frc.robot.subsystems.rollers.intake.IntakeIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.arm.ArmPositionPID;
// import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShooterIO;
import frc.robot.subsystems.superstructure.shooter.ShooterIOSim;
import frc.robot.subsystems.superstructure.shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVision;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVisionSIM;
import frc.robot.util.FieldConstants;
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
  private static DriveController driveMode = new DriveController();
  private Intake intake;
  private Feeder feeder1;
  private Feeder feeder2;
  private Rollers rollers;

  private Superstructure superstructure;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private ArmPositionPID armPID = new ArmPositionPID();
  // private final Climber climberPID = new Climber();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private static final Transform3d robotToCameraBL =
  new Transform3d(
      new Translation3d(Units.inchesToMeters(-13.), Units.inchesToMeters(12.), Units.inchesToMeters(10.)),
      new Rotation3d(0, Math.toRadians(-15.), Math.toRadians(225.)));

private static final Transform3d robotToCameraBR =
  new Transform3d(
      new Translation3d(Units.inchesToMeters(-13.), Units.inchesToMeters(-12.), Units.inchesToMeters(10.)),
      new Rotation3d(0, Math.toRadians(-15.), Math.toRadians(315.)));

private static final Transform3d robotToCameraFront =
  new Transform3d(
      new Translation3d(Units.inchesToMeters(11), Units.inchesToMeters(12.), Units.inchesToMeters(10.)),
      new Rotation3d(0, Math.toRadians(-15), Math.toRadians(0.)));


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkFlex(moduleConfigs[0]),
                new ModuleIOSparkFlex(moduleConfigs[1]),
                new ModuleIOSparkFlex(moduleConfigs[2]),
                new ModuleIOSparkFlex(moduleConfigs[3]));
        shooter = new Shooter(new ShooterIOSparkFlex());
        feeder1 = new Feeder(new FeederIOSparkFlexFront());
        feeder2 = new Feeder(new FeederIOSparkFlexBack());
        intake = new Intake(new IntakeIOSparkFlex());
        superstructure = new Superstructure(shooter);

        aprilTagVision =
            new AprilTagVision(
                // new AprilTagVisionIOPhotonVision("BLCamera", robotToCameraBL),
                new AprilTagVisionIOPhotonVision("BRCamera", robotToCameraBR),
                new AprilTagVisionIOPhotonVision("FrontCamera", robotToCameraFront));
        rollers = new Rollers(feeder1, feeder2, intake);

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
        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOPhotonVisionSIM(
                    "photonCamera1",
                    new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)),
                    drive::getDrive));

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

    // Set up auto routines
    NamedCommands.registerCommand(
        "Shoot",
        Commands.sequence(
            Commands.runOnce(
                () -> superstructure.setGoal(Superstructure.SystemState.PREPARE_SHOOT),
                superstructure),
            Commands.startEnd(
                    () -> DriveCommands.setSpeakerMode(drive::getPose),
                    DriveCommands::disableDriveHeading)
                .alongWith(
                    new MultiDistanceArm(
                        drive::getPose, FieldConstants.Speaker.centerSpeakerOpening, armPID)),
            Commands.waitSeconds(1),
            Commands.runOnce(
                () -> superstructure.setGoal(Superstructure.SystemState.SHOOT), superstructure),
            Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.SHOOT), rollers),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  shooter.setGoal(Shooter.Goal.IDLE);
                  superstructure.setGoal(Superstructure.SystemState.IDLE);
                })));

    NamedCommands.registerCommand(
        "Intake",
        Commands.sequence(
            Commands.runOnce(
                () -> superstructure.setGoal(Superstructure.SystemState.INTAKE), superstructure),
            Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.FLOOR_INTAKE), rollers),
            Commands.waitUntil(() -> !rollers.getBeamBreak()),
            Commands.runOnce(
                () -> {
                  shooter.setGoal(Shooter.Goal.IDLE);
                  superstructure.setGoal(Superstructure.SystemState.IDLE);
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

    // Configure the button bindings
    aprilTagVision.setDataInterfaces(drive::addVisionData);
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
    // ==================
    // DEFAULT COMMANDS
    // ==================
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    driverController.leftBumper().whileTrue(Commands.runOnce(() -> driveMode.toggleDriveMode()));

    // +++++++++++++++++++++THIS_IS_THE_CODE_FOR_THE_MULTIPLE_DISTANCE_SHOT++++++++++++++++++++++++++++
    // driverController
    // .rightTrigger()
    // .whileTrue(
    // new MultiDistanceShot(
    // drive::getPose, FieldConstants.Speaker.centerSpeakerOpening, shooter));

    // ================================================
    // DRIVER CONTROLLER - LEFT BUMPER
    // RUN INTAKE IN
    // ================================================
    driverController
        .leftBumper()
        .whileTrue( // Tyler Fixed This. :)
            Commands.sequence(
                Commands.runOnce(
                    () -> superstructure.setGoal(Superstructure.SystemState.INTAKE),
                    superstructure),
                Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.FLOOR_INTAKE), rollers),
                Commands.waitUntil(() -> !rollers.getBeamBreak()),
                Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.EJECTALIGN)),
                Commands.waitUntil(() -> rollers.getBeamBreak()),
                Commands.runOnce(
                    () -> {
                      rollers.setGoal(Rollers.Goal.IDLE);
                      superstructure.setGoal(Superstructure.SystemState.IDLE);
                    })))
        .onFalse(
            Commands.runOnce(
                () -> {
                  rollers.setGoal(Rollers.Goal.IDLE);
                  superstructure.setGoal(Superstructure.SystemState.IDLE);
                }));

    // ================================================
    // DRIVER CONTROLLER - LEFT TRIGGER
    // RUN INTAKE OUT
    // ================================================
    driverController
        .leftTrigger()
        .whileTrue(
            Commands.runOnce(
                    () -> superstructure.setGoal(Superstructure.SystemState.INTAKE), superstructure)
                .andThen(
                    Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.EJECT_TO_FLOOR), rollers),
                    Commands.idle())
                .finallyDo(
                    () -> {
                      rollers.setGoal(Rollers.Goal.IDLE);
                      superstructure.setGoal(Superstructure.SystemState.IDLE);
                    }));

    // ================================================
    // DRIVER CONTROLLER - LEFT BUMPER
    // SET DRIVE MODE TO AMP
    // ================================================
    driverController
        .rightBumper()
        .whileTrue(Commands.runOnce(() -> driveMode.setDriveMode(DriveModeType.AMP)));

    // ================================================
    // DRIVER CONTROLLER - A
    // PATHFIND TO SELECTED DRIVE MODE
    // ================================================
    driverController.a().whileTrue(new PathFinderAndFollow(driveMode.getDriveModeType()));
    ;

    // ================================================
    // DRIVER CONTROLLER - START
    // SET AUTO START POSE (i think it sets the heading)
    // ================================================
    driverController
        .start()
        .whileTrue(
            Commands.run(
                () ->
                    drive.setAutoStartPose(
                        new Pose2d(new Translation2d(10, 1), Rotation2d.fromDegrees(0)))));

    // ================================================
    // DRIVER CONTROLLER - DPAD UP
    // MOVE CLIMBER UP
    // ================================================
    // driverController.povUp().whileTrue(new PositionClimbPID(climberPID, 20));

    // ================================================
    // DRIVER CONTROLLER - DPAD DOWN
    // MOVE CLIMBER DOWN
    // ================================================
    // driverController.povDown().whileTrue(new PositionClimbPID(climberPID, 0));

    // AMP LOCATION
    // operatorController.leftBumper().whileTrue(new PositionArmPID(armPID, 250));
    // AMP SCORE
    operatorController
        .leftBumper()
        .whileTrue(
            Commands.sequence(
                    Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.AMP_SHOOTER), rollers))
                .alongWith(new PositionArmPID(armPID, 175)))
        .onFalse(
            Commands.runOnce(
                () -> {
                  rollers.setGoal(Rollers.Goal.IDLE);
                  superstructure.setGoal(Superstructure.SystemState.IDLE);
                }));

    // ================================================
    // OPERATOR CONTROLLER - RIGHT BUMPER/TRIGGER
    // BUMPER CHARGES SHOOTER, TRIGGER SHOOTS
    // ================================================
    operatorController
        .rightBumper()
        .whileTrue( // Yousef and Toby Fixed This. :)
            Commands.sequence(
                Commands.runOnce(
                    () -> superstructure.setGoal(Superstructure.SystemState.PREPARE_SHOOT),
                    superstructure),
                Commands.waitUntil(operatorController.rightTrigger()),
                Commands.runOnce(
                    () -> superstructure.setGoal(Superstructure.SystemState.SHOOT), superstructure),
                Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.SHOOT), rollers),
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> {
                      shooter.setGoal(Shooter.Goal.IDLE);
                      superstructure.setGoal(Superstructure.SystemState.IDLE);
                    })))
        .onFalse(
            Commands.runOnce(
                () -> {
                  rollers.setGoal(Rollers.Goal.IDLE);
                  superstructure.setGoal(Superstructure.SystemState.IDLE);
                }));

    // ================================================
    // OPERATOR CONTROLLER - LEFT TRIGGER
    // AIM SHOOTER AT SPEAKER
    // ================================================
    operatorController
        .leftTrigger()
        .whileTrue(
            Commands.startEnd(
                    () -> DriveCommands.setSpeakerMode(drive::getPose),
                    DriveCommands::disableDriveHeading)
                .alongWith(
                    new MultiDistanceArm(
                        drive::getPose, FieldConstants.Speaker.centerSpeakerOpening, armPID)));

    // controller
    // .povDown()
    // .whileTrue(
    // new ShootPoint(
    // drive, new Pose2d(new Translation2d(2.954, 3.621),
    // Rotation2d.fromRadians(2.617))));

    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // ================================================
    // OPERATOR CONTROLLER - DPAD UP
    // ARM POSITION SUB SHOOT
    // ================================================
    operatorController.povUp().whileTrue(new PositionArmPID(armPID, 200.00)); // "Sub shoot"

    // ================================================
    // OPERATOR CONTROLLER - DPAD RIGHT
    // ARM POSITION MIDFIELD SHOOT
    // ================================================
    operatorController.povRight().whileTrue(new PositionArmPID(armPID, 100.0)); // "Midfield Shoot"

    // ================================================
    // OPERATOR CONTROLLER - DPAD LEFT
    // ARM POSITION AMP SHOOT
    // ================================================
    operatorController.povLeft().whileTrue(new PositionArmPID(armPID, 175.0)); // "Amp/Note Shoot"

    // ================================================
    // OPERATOR CONTROLLER - DPAD DOWN
    // ARM POSITION PILLAR SHOOT
    // ================================================
    operatorController.povDown().whileTrue(new PositionArmPID(armPID, 0.0)); // "Pillar Shoot"
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
