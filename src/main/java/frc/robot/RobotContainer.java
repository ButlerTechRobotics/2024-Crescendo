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
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MultiDistanceArm;
import frc.robot.commands.PathFinderAndFollow;
// import frc.robot.commands.ShootDistance;
import frc.robot.commands.arm.PositionArmPID;
import frc.robot.commands.climber.PositionClimbPID;
// import frc.robot.subsystems.SwagLights;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
<<<<<<< HEAD
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
=======
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIONeo;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
>>>>>>> Sonic
import frc.robot.subsystems.leds.Candle;
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
import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShooterIO;
import frc.robot.subsystems.superstructure.shooter.ShooterIOSim;
import frc.robot.subsystems.superstructure.shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVision;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVisionSIM;
import frc.robot.util.AllianceFlipUtil;
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

  private Candle candle = new Candle();

  private Superstructure superstructure;

  private boolean hasRun = false;
  private boolean hasEjected = false; // New flag for the EJECTALIGN command

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private ArmPositionPID armPID = new ArmPositionPID();
  private final Climber climberPID = new Climber();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

<<<<<<< HEAD
  private static final Transform3d robotToCameraFL =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-14.75), Units.inchesToMeters(10.75), Units.inchesToMeters(9.5)),
          new Rotation3d(0, Math.toRadians(-28.), Math.toRadians(150.)));

  private static final Transform3d robotToCameraFR =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-14.75),
              Units.inchesToMeters(-10.75),
              Units.inchesToMeters(9.5)),
=======
  private static final Transform3d robotToCameraBL =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.5), Units.inchesToMeters(11.5), Units.inchesToMeters(9.5)),
          new Rotation3d(0, Math.toRadians(-28.), Math.toRadians(150.)));

  private static final Transform3d robotToCameraBR =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.5), Units.inchesToMeters(-11.5), Units.inchesToMeters(9.5)),
>>>>>>> Sonic
          new Rotation3d(0, Math.toRadians(-28.), Math.toRadians(-150.)));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware I` implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new SwerveModuleIONeo(moduleConfigs[0]),
                new SwerveModuleIONeo(moduleConfigs[1]),
                new SwerveModuleIONeo(moduleConfigs[2]),
                new SwerveModuleIONeo(moduleConfigs[3]));

        shooter = new Shooter(new ShooterIOSparkFlex());
        superstructure = new Superstructure(shooter);

        feeder1 = new Feeder(new FeederIOSparkFlexFront());
        feeder2 = new Feeder(new FeederIOSparkFlexBack());
        intake = new Intake(new IntakeIOSparkFlex());
        rollers = new Rollers(feeder1, feeder2, intake);

        aprilTagVision =
            new AprilTagVision(
<<<<<<< HEAD
                new AprilTagVisionIOPhotonVision("FLCamera", robotToCameraFL),
                new AprilTagVisionIOPhotonVision("FRCamera", robotToCameraFR));
=======
                new AprilTagVisionIOPhotonVision("BLCamera", robotToCameraBL),
                new AprilTagVisionIOPhotonVision("BRCamera", robotToCameraBR));
>>>>>>> Sonic
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim());

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
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {});
        shooter = new Shooter(new ShooterIO() {});

        feeder1 = new Feeder(new FeederIO() {});

        feeder2 = new Feeder(new FeederIO() {});

        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});

        intake = new Intake(new IntakeIO() {});
    }

    // ================================================
    // Register the Auto Command Gyro
    // ================================================

    NamedCommands.registerCommand(
        "Gyro Reset",
        Commands.run(
            () ->
                drive.setAutoStartPose(
                    new Pose2d(new Translation2d(15.312, 5.57), Rotation2d.fromDegrees(0)))));

    // ================================================
    // Register the Auto Command PreShoot
    // ================================================
    NamedCommands.registerCommand(
        "PreShoot",
        Commands.runOnce(
            () -> superstructure.setGoal(Superstructure.SystemState.PREPARE_SHOOT),
            superstructure));

    // ================================================
    // Register the Auto Command ShooterPosLeft
    // ================================================
    NamedCommands.registerCommand(
<<<<<<< HEAD
        "ShooterPosLeft", Commands.runOnce(() -> armPID.setPosition(4.5)));
=======
        "ShooterPosLeft", Commands.runOnce(() -> armPID.setPosition(11.4878)));
>>>>>>> Sonic

    // ================================================
    // Register the Auto Command Shooter Reset
    // ================================================
<<<<<<< HEAD
    NamedCommands.registerCommand("Shooter Reset", Commands.runOnce(() -> armPID.setPosition(4.5)));
=======
    NamedCommands.registerCommand("Shooter Reset", Commands.runOnce(() -> armPID.setPosition(2.8)));
>>>>>>> Sonic

    // ================================================
    // Register the Auto Command Shoot
    // ================================================
    NamedCommands.registerCommand(
        "Shoot",
        Commands.sequence(
            Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.SHOOT), rollers),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  shooter.setGoal(Shooter.Goal.IDLE);
                  superstructure.setGoal(Superstructure.SystemState.IDLE);
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
    // Register the Auto Command Heading Reset
    // ================================================
    NamedCommands.registerCommand(
        "Heading Reset",
        Commands.runOnce(
            () ->
                drive.setAutoStartPose(
                    new Pose2d(new Translation2d(15.312, 5.57), Rotation2d.fromDegrees(0)))));

    // ================================================
    // Register the Auto Command Intake
    // ================================================
    NamedCommands.registerCommand(
        "Intake",
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (!hasRun) {
                    superstructure.setGoal(Superstructure.SystemState.INTAKE);
                    rollers.setGoal(Rollers.Goal.FLOOR_INTAKE);
                    hasRun = true;
                  }
                },
                superstructure,
                rollers),
            Commands.waitUntil(() -> !rollers.getBeamBreak()),
            Commands.runOnce(
                () -> {
                  if (!hasEjected) {
                    rollers.setGoal(Rollers.Goal.EJECTALIGN);
                    hasEjected = true;
                  }
                },
                rollers),
            Commands.waitUntil(() -> rollers.getBeamBreak()),
            Commands.runOnce(
                () -> {
                  rollers.setGoal(Rollers.Goal.IDLE);
                  superstructure.setGoal(Superstructure.SystemState.IDLE);
                })));

    // ================================================
    // Register the Auto Command ShooterPosLeft
    // ================================================
    NamedCommands.registerCommand(
        "ArmPositionAmp", Commands.runOnce(() -> armPID.setPosition(78.0)));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
    // ==================
    // DEFAULT COMMANDS
    // ==================
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            driveMode,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    driverController.back().whileTrue(Commands.runOnce(() -> driveMode.toggleDriveMode()));

    // ================================================
    // DRIVER CONTROLLER - LEFT BUMPER
    // RUN INTAKE IN
    // ================================================
    driverController
        .leftBumper()
        .whileTrue( // Tyler Fixed This. :)
            Commands.sequence(
<<<<<<< HEAD
                candle.runBurnyBurnCommand(),
=======
                candle.runPrettyLightsCommand(),
>>>>>>> Sonic
                Commands.runOnce(
                    () -> superstructure.setGoal(Superstructure.SystemState.INTAKE),
                    superstructure),
                Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.FLOOR_INTAKE), rollers),
                Commands.waitUntil(() -> !rollers.getBeamBreak()),
                Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.EJECTALIGN)),
                candle.setColorGreenCommand(),
                Commands.waitUntil(() -> rollers.getBeamBreak()),
                Commands.runOnce(
                    () -> {
                      rollers.setGoal(Rollers.Goal.IDLE);
                      superstructure.setGoal(Superstructure.SystemState.IDLE);
                    }),
<<<<<<< HEAD
                candle.setColorOperationIdle()))
=======
                Commands.waitSeconds(0.5),
                candle.setColorRespawnIdle()))
>>>>>>> Sonic
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
    // DRIVER CONTROLLER - A
    // PATHFIND TO SELECTED DRIVE MODE
    // ================================================
    driverController.a().whileTrue(new PathFinderAndFollow(driveMode.getDriveModeType()));

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
                        new Pose2d(new Translation2d(15.312, 5.57), Rotation2d.fromDegrees(180)))));

    // ================================================
    // DRIVER CONTROLLER - DPAD UP
    // MOVE CLIMBER UP
    // ================================================
<<<<<<< HEAD
    driverController.povUp().whileTrue(new PositionClimbPID(climberPID, 3000));
=======
    driverController.povUp().whileTrue(new PositionClimbPID(climberPID, 300));
>>>>>>> Sonic

    // ================================================
    // DRIVER CONTROLLER - DPAD DOWN
    // MOVE CLIMBER DOWN
    // ================================================
<<<<<<< HEAD
    driverController.povDown().whileTrue(new PositionClimbPID(climberPID, -1000));
=======
    driverController.povDown().whileTrue(new PositionClimbPID(climberPID, -300));
>>>>>>> Sonic

    // AMP LOCATION
    // operatorController.leftBumpeSCOREr().whileTrue(new PositionArmPID(armPID,
    // 250));
    // AMP
    operatorController
        .leftBumper()
        .whileTrue(
            Commands.sequence(
<<<<<<< HEAD
                    Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.AMP_SHOOTER), rollers))
                .alongWith(new PositionArmPID(armPID, 89)))
=======
                Commands.runOnce(() -> rollers.setGoal(Rollers.Goal.AMP_SHOOTER), rollers)))
>>>>>>> Sonic
        .onFalse(
            Commands.runOnce(
                () -> {
                  rollers.setGoal(Rollers.Goal.IDLE);
                }));

    // ================================================
    // OPERATOR CONTROLLER - RIGHT BUMPER/TRIGGER
    // BUMPER CHARGES SHOOTER, TRIGGER SHOOTS
    // ================================================
    operatorController
        .rightBumper()
        .whileTrue( // Yousef and Toby Fixed This. :)
            Commands.sequence(
                candle.runPrepareShootCommand(),
                Commands.runOnce(
                    () -> superstructure.setGoal(Superstructure.SystemState.PREPARE_SHOOT),
                    superstructure),
                Commands.waitUntil(operatorController.rightTrigger()),
                candle.runShootCommand(),
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
                    })
<<<<<<< HEAD
                .alongWith(candle.setColorOperationIdle()));
=======
                .alongWith(candle.setColorRespawnIdle()));
>>>>>>> Sonic

    // ================================================
    // OPERATOR CONTROLLER - LEFT TRIGGER
    // AIM SHOOTER AT SPEAKER
    // ================================================
    operatorController
        .leftTrigger()
        .whileTrue(
            Commands.startEnd(
                    () -> driveMode.enableHeadingControl(), () -> driveMode.disableHeadingControl())
                .alongWith(
                    new MultiDistanceArm(
                        drive::getPose,
                        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening),
                        armPID)));
    // driverController
    // .rightBumper()
    // .whileTrue(
    // Commands.startEnd(
    // () -> driveMode.enableHeadingControl(), () ->
    // driveMode.disableHeadingControl()));

    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    PathConstraints pathConstraints = new PathConstraints(4, 4, 540, 720);

    SmartDashboard.putData(
        "Pathfind to Trap Score 3",
        AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("TrapScore3"), pathConstraints));

    // ================================================
    // OPERATOR CONTROLLER - DPAD UP
    // ARM POSITION SUB SHOOT
    // ================================================
<<<<<<< HEAD
    operatorController.povUp().whileTrue(new PositionArmPID(armPID, 105)); // "Sub shoot"
=======
    operatorController.povUp().whileTrue(new PositionArmPID(armPID, 96.0 + 2.8)); // "Sub shoot"
>>>>>>> Sonic

    // ================================================
    // OPERATOR CONTROLLER - DPAD RIGHT
    // ARM POSITION MIDFIELD SHOOT
    // ================================================
<<<<<<< HEAD
    operatorController.povRight().whileTrue(new PositionArmPID(armPID, 8.5)); // "Midfield Shoot"
=======
    operatorController
        .povRight()
        .whileTrue(
            new PositionArmPID(
                armPID, 17.0)); // "Stage Shoot" // Was -16.25 and shot a little too high
>>>>>>> Sonic

    // ================================================
    // OPERATOR CONTROLLER - DPAD LEFT
    // ARM POSITION AMP SHOOT
    // ================================================
<<<<<<< HEAD
    operatorController
        .povLeft()
        .whileTrue(new PositionArmPID(armPID, 89)); // "Amp/Note Shoot" -50!!!!
=======
    operatorController.povLeft().whileTrue(new PositionArmPID(armPID, 78)); // "Amp/Note Shoot"
>>>>>>> Sonic

    // ================================================
    // OPERATOR CONTROLLER - DPAD DOWN
    // ARM POSITION PILLAR SHOOT
    // ================================================
<<<<<<< HEAD
    operatorController.povDown().whileTrue(new PositionArmPID(armPID, 4.25)); // "Pillar Shoot"
=======
    operatorController.povDown().whileTrue(new PositionArmPID(armPID, 0.5)); // "Pillar Shoot"
>>>>>>> Sonic
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
