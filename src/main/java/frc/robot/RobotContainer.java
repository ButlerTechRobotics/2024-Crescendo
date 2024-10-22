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
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        public static Drive drive;
        private Shooter shooter;
        private AprilTagVision aprilTagVision;
        private Arm arm;
        private Intake intake;
        private BeamBreak beamBreak;
        private Magazine magazine;
        private Candle candle;

        // Controller
        private final CommandXboxController driverController = new CommandXboxController(0);
        private final CommandXboxController demoController = new CommandXboxController(1);

        private final ClimberLeft climberLeft = new ClimberLeft();
        private final ClimberRight climberRight = new ClimberRight();

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.getMode()) {
                        case REAL:
                                // Real robot, instantiate hardware I` implementations
                                drive = new Drive(
                                                new GyroIOPigeon2(),
                                                new ModuleIOTalonFX(moduleConfigs[0]),
                                                new ModuleIOTalonFX(moduleConfigs[1]),
                                                new ModuleIOTalonFX(moduleConfigs[2]),
                                                new ModuleIOTalonFX(moduleConfigs[3]));

                                shooter = new Shooter(new ShooterIOSparkFlex());
                                arm = new Arm(new ArmIONeo());
                                magazine = new Magazine(new MagazineIONeo());
                                beamBreak = new BeamBreak(new BeamBreakIODigitalInput());
                                intake = new Intake(new IntakeIONeo());

                                aprilTagVision = new AprilTagVision(
                                                new AprilTagVisionIOPhotonVision("BLCamera", ROBOT_TO_CAMERA_BL),
                                                new AprilTagVisionIOPhotonVision("BRCamera", ROBOT_TO_CAMERA_BR),
                                                new AprilTagVisionIOPhotonVision("BackCamera", ROBOT_TO_CAMERA_BACK));
                                candle = new Candle(aprilTagVision);
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim());

                                shooter = new Shooter(new ShooterIOSim());
                                arm = new Arm(new ArmIOSim());
                                magazine = new Magazine(new MagazineIOSim());
                                beamBreak = new BeamBreak(new BeamBreakIOSim());
                                intake = new Intake(new IntakeIOSim());

                                aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {
                                });
                                candle = new Candle(aprilTagVision);

                                break;

                        default:
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                });
                                shooter = new Shooter(new ShooterIO() {
                                });

                                arm = new Arm(new ArmIO() {
                                });

                                magazine = new Magazine(new MagazineIO() {
                                });

                                beamBreak = new BeamBreak(new BeamBreakIO() {
                                });

                                intake = new Intake(new IntakeIO() {
                                });

                                aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {
                                });

                                candle = new Candle(aprilTagVision);
                }
                // ================================================
                // Register the Named Command Intake
                // ================================================
                NamedCommands.registerCommand("Intake", new ManualIntake(intake, magazine, beamBreak));

                // ================================================
                // Register the Named Command Shoot
                // ================================================
                NamedCommands.registerCommand(
                                "Shoot",
                                new ManualShoot(shooter, magazine, beamBreak, 0.0)
                                                .andThen(
                                                                Commands.runOnce(
                                                                                () -> {
                                                                                        shooter.setSetpoint(0, 0);
                                                                                        arm.setArmTargetAngle(
                                                                                                        ArmConstants.home
                                                                                                                        .arm()
                                                                                                                        .getDegrees());
                                                                                })));

                // ================================================
                // Register the Named Command QuickShoot
                // ================================================
                NamedCommands.registerCommand(
                                "QuickShoot",
                                new SmartShoot(arm, shooter, magazine, beamBreak, drive::getPose, 1.0)
                                                .deadlineWith(DriveCommands.joystickDrive(drive, () -> 0, () -> 0,
                                                                () -> 0))
                                                .andThen(
                                                                new ScheduleCommand(
                                                                                Commands.defer(() -> new ShotVisualizer(
                                                                                                drive, arm, shooter),
                                                                                                Set.of()))));

                // // ================================================
                // // Register the Named Command EnableSmartSpeaker
                // // ================================================
                NamedCommands.registerCommand(
                                "EnableSmartSpeaker",
                                Commands.sequence(
                                                Commands.runOnce(
                                                                () -> SmartController.getInstance()
                                                                                .setDriveMode(DriveModeType.SPEAKER)),
                                                Commands.runOnce(SmartController.getInstance()::enableSmartControl)));

                NamedCommands.registerCommand(
                                "EnableSmartControl",
                                Commands.runOnce(SmartController.getInstance()::enableSmartControl));

                // // ================================================
                // // Register the Named Command DisableSmartControl
                // // ================================================
                // NamedCommands.registerCommand(
                // "DisableSmartControl",
                // Commands.runOnce(SmartController.getInstance()::disableSmartControl));

                // // ================================================
                // // Register the Named Command SmartShoot
                // // ================================================
                NamedCommands.registerCommand(
                                "SmartShoot",
                                Commands.sequence(
                                                Commands.runOnce(
                                                                () -> SmartController.getInstance()
                                                                                .setDriveMode(DriveModeType.SPEAKER)),
                                                new SmartShoot(arm, shooter, magazine, beamBreak, drive::getPose, 1.5),
                                                Commands.runOnce(SmartController.getInstance()::disableSmartControl)));

                // =========================%=======================
                // Register the Named Command SmartIntake
                // ================================================
                NamedCommands.registerCommand(
                                "SmartIntake", new SmartIntake(intake, beamBreak, magazine, candle));

                // =========================%=======================
                // Register the Named Command SmartIntake
                // ================================================
                NamedCommands.registerCommand(
                                "SmartControl",
                                Commands.parallel(
                                                new SmartShooter(shooter),
                                                new SmartArm(arm),
                                                new SmartIntake(intake, beamBreak, magazine, candle)));

                NamedCommands.registerCommand(
                                "PreRollShoot",
                                Commands.deadline(
                                                new SmartShoot(arm, shooter, magazine, beamBreak, drive::getPose, 1.5),
                                                new SmartShooter(shooter),
                                                new SmartArm(arm),
                                                DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0))
                                                .andThen(
                                                                Commands.runOnce(
                                                                                () -> {
                                                                                        shooter.setSetpoint(0, 0);
                                                                                        arm.setArmTargetAngle(
                                                                                                        ArmConstants.home
                                                                                                                        .arm()
                                                                                                                        .getDegrees());
                                                                                })));

                NamedCommands.registerCommand(
                                "PreRollShootAndMove",
                                Commands.deadline(
                                                new SmartShoot(arm, shooter, magazine, beamBreak, drive::getPose, 1.5),
                                                new SmartShooter(shooter),
                                                new SmartArm(arm))
                                                .andThen(
                                                                Commands.runOnce(
                                                                                () -> {
                                                                                        shooter.setSetpoint(0, 0);
                                                                                        arm.setArmTargetAngle(
                                                                                                        ArmConstants.home
                                                                                                                        .arm()
                                                                                                                        .getDegrees());
                                                                                })));

                NamedCommands.registerCommand(
                                "ManualUpCloseShot",
                                Commands.sequence(new ManualShoot(shooter, magazine, beamBreak, 1)));

                NamedCommands.registerCommand(
                                "PreRollShootFast",
                                Commands.deadline(
                                                new SmartShoot(arm, shooter, magazine, beamBreak, drive::getPose, 0.5),
                                                new SmartShooter(shooter),
                                                new SmartArm(arm))
                                                .andThen(
                                                                Commands.runOnce(
                                                                                () -> {
                                                                                        shooter.setSetpoint(0, 0);
                                                                                        arm.setArmTargetAngle(
                                                                                                        ArmConstants.home
                                                                                                                        .arm()
                                                                                                                        .getDegrees());
                                                                                })));

                NamedCommands.registerCommand(
                                "PreRollShootInstant",
                                Commands.deadline(
                                                new SmartShoot(arm, shooter, magazine, beamBreak, drive::getPose, 0.0),
                                                new SmartShooter(shooter),
                                                new SmartArm(arm))
                                                .andThen(
                                                                Commands.runOnce(
                                                                                () -> {
                                                                                        shooter.setSetpoint(0, 0);
                                                                                        arm.setArmTargetAngle(
                                                                                                        ArmConstants.home
                                                                                                                        .arm()
                                                                                                                        .getDegrees());
                                                                                })));

                NamedCommands.registerCommand("Magazine", new ManualMagazine(magazine, beamBreak));

                NamedCommands.registerCommand(
                                "Preload", new InstantCommand(() -> beamBreak.setGamePiece(true)));

                NamedCommands.registerCommand(
                                "PodiumPreroll",
                                new AutoPreRoll(arm, shooter, beamBreak, Rotation2d.fromDegrees(158), 4500));

                NamedCommands.registerCommand(
                                "ClosePreroll",
                                new AutoPreRoll(arm, shooter, beamBreak, Rotation2d.fromDegrees(129), 2500));

                NamedCommands.registerCommand(
                                "PB3AC Preroll",
                                new AutoPreRoll(arm, shooter, beamBreak, Rotation2d.fromDegrees(141), 3150));

                NamedCommands.registerCommand(
                                "AS Far Preroll",
                                new AutoPreRoll(arm, shooter, beamBreak, Rotation2d.fromDegrees(162.8), 4500));

                NamedCommands.registerCommand(
                                "AS Mid Preroll",
                                new AutoPreRoll(arm, shooter, beamBreak, Rotation2d.fromDegrees(162.8), 4500));

                NamedCommands.registerCommand(
                                "PodiumShot",
                                new AutoPreRoll(arm, shooter, beamBreak, Rotation2d.fromDegrees(129), 2500));

                NamedCommands.registerCommand(
                                "PB3AC Note B",
                                new AutoPreRoll(arm, shooter, beamBreak, Rotation2d.fromDegrees(151.5), 3700)
                                                .andThen(Commands.waitSeconds(0.5))
                                                .andThen());

                NamedCommands.registerCommand(
                                "MiniBlurp Preroll",
                                new AutoPreRoll(arm, shooter, beamBreak, Rotation2d.fromDegrees(129), 2000)
                                                .andThen(Commands.waitSeconds(0.5))
                                                .andThen());

                // AUTON PATHS ========================

                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

                autoChooser.addOption(
                                "Drive FF Characterization",
                                new FeedForwardCharacterization(
                                                drive, drive::runCharacterizationVolts,
                                                drive::getCharacterizationVelocity));

                autoChooser.addOption(
                                "Wheel Radius Characterization",
                                Commands.run(drive::setWheelsToCircle)
                                                .withTimeout(2)
                                                .andThen(new WheelRadiusCharacterization(drive)));

                // ================================================
                // 4 Note Amp P-1-2-3
                // ================================================
                Command fourNoteAmpP123 = Commands.sequence(
                                new PathPlannerAuto("Amp P-1-2-3 Drop P Collect 1"), // Drop P, collect 1
                                Commands.either(
                                                new PathPlannerAuto("Amp P-1-2-3 Score 1 Collect 2"), // Score 1,
                                                // collect 2
                                                new PathPlannerAuto("Amp P-1-2-3 Missed 1 Collect 2"), // Missed 1,
                                                // collect 2
                                                beamBreak::hasNoteForAuto),
                                Commands.either(
                                                new PathPlannerAuto("Amp P-1-2-3 Score 2 Collect 3"), // Score 2,
                                                // collect 3
                                                new PathPlannerAuto("Amp P-1-2-3 Missed 2 Collect 3"), // Missed 2,
                                                // collect 3
                                                beamBreak::hasNoteForAuto),
                                Commands.either(
                                                new PathPlannerAuto("Amp P-1-2-3 Score 3 Collect P"), // Score 3,
                                                // collect P
                                                new PathPlannerAuto("Amp P-1-2-3 Missed 3 Collect P"), // Missed 3,
                                                // collect P
                                                beamBreak::hasNoteForAuto),
                                Commands.either(
                                                new PathPlannerAuto("Amp P-1-2-3 Score P"), // Score P
                                                new PathPlannerAuto("Amp P-1-2-3 Missed P Sprint Source"),
                                                beamBreak::hasNoteForAuto));
                autoChooser.addOption("4 Note Amp P-1-2-3", fourNoteAmpP123);

                // ================================================
                // 3 Note Source P-5-4
                // ================================================
                Command threeNoteSourceP54 = Commands.sequence(
                                new PathPlannerAuto("Source Drop P Collect 5"), // Drop P, collect 1
                                Commands.either(
                                                new PathPlannerAuto("Source Score 5 Collect 4"), // Score 1,
                                                // collect 2
                                                new PathPlannerAuto("Source Missed 5 Collect 4"), // Missed 1,
                                                // collect 2
                                                beamBreak::hasNoteForAuto),
                                Commands.either(
                                                new PathPlannerAuto("Source Score 4 Collect P"), // Score 2,
                                                // collect 3
                                                new PathPlannerAuto("Source Missed 4 Collect P"), // Missed 2,
                                                // collect 3
                                                beamBreak::hasNoteForAuto),
                                Commands.either(
                                                new PathPlannerAuto("Amp P-1-2-3 Score P"), // Score P
                                                new PathPlannerAuto("Amp P-1-2-3 Missed P Sprint Source"),
                                                beamBreak::hasNoteForAuto));
                autoChooser.addOption("3 Note Source P-5-4", threeNoteSourceP54);

                // Run SmartController updates in autonomous
                new Trigger(DriverStation::isAutonomousEnabled)
                                .and(
                                                new Trigger(
                                                                () -> SmartController.getInstance()
                                                                                .getDriveModeType() == DriveModeType.SPEAKER))
                                .whileTrue(
                                                new InstantCommand(
                                                                () -> {
                                                                        SmartController.getInstance()
                                                                                        .calculateSpeaker(
                                                                                                        drive.getPose(),
                                                                                                        new Translation2d(
                                                                                                                        0,
                                                                                                                        0),
                                                                                                        new Translation2d(
                                                                                                                        0,
                                                                                                                        0));
                                                                }));

                // Configure the button bindings
                aprilTagVision.setDataInterfaces(drive::addVisionData);
                SmartController.getInstance().disableSmartControl();

                configureButtonBindings();
        }

        public Command manualBlurp() {
                return Commands.sequence(
                                Commands.runOnce(() -> shooter.manualBlurp()),
                                Commands.waitSeconds(0.5),
                                Commands.runOnce(() -> magazine.shoot()),
                                Commands.runOnce(() -> shooter.setSetpoint(0, 0)));
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
                candle.setDefaultCommand(new HandleCandle(candle, beamBreak));
                intake.setDefaultCommand(
                                new SmartIntake(intake, beamBreak, magazine, candle).ignoringDisable(true));
                magazine.setDefaultCommand(new SmartMagazine(magazine, intake, beamBreak, candle));

                // ================================================
                // DRIVER CONTROLLER - START
                // RESET HEADING
                // ================================================
                driverController
                                .start()
                                .whileTrue(
                                                Commands.run(
                                                                () -> drive.setPose(
                                                                                new Pose2d(new Translation2d(15.312,
                                                                                                5.57),
                                                                                                Rotation2d.fromDegrees(
                                                                                                                180)))));

                // ================================================
                // DRIVER CONTROLLER - LEFT BUMPER
                // RUN INTAKE IN
                // ================================================
                driverController
                                .leftBumper()
                                .whileTrue(Commands.startEnd(intake::enableIntakeRequest,
                                                intake::disableIntakeRequest));

                // ================================================
                // DRIVER CONTROLLER - LEFT TRIGGER
                // RUN INTAKE OUT
                // ================================================
                driverController
                                .leftTrigger()
                                .whileTrue(Commands.startEnd(intake::enableOuttakeRequest,
                                                intake::disableOuttakeRequest));

                // ================================================
                // DRIVER CONTROLLER - A
                // SET DRIVE MODE TO AMP
                // ================================================
                driverController
                                .a()
                                .onTrue(
                                                Commands.runOnce(() -> SmartController.getInstance()
                                                                .setDriveMode(DriveModeType.AMP))
                                                                .alongWith(
                                                                                Commands.runOnce(() -> SmartController
                                                                                                .getInstance()
                                                                                                .enableSmartControl())));

                // ================================================
                // DRIVER CONTROLLER - B
                // SET DRIVE MODE TO SPEAKER
                // ================================================
                driverController
                                .b()
                                .onTrue(
                                                Commands.runOnce(
                                                                () -> SmartController.getInstance()
                                                                                .setDriveMode(DriveModeType.SPEAKER))
                                                                .alongWith(
                                                                                Commands.runOnce(() -> SmartController
                                                                                                .getInstance()
                                                                                                .enableSmartControl())));

                // ================================================
                // DRIVER CONTROLLER - X
                // SET DRIVE MODE TO SAFE
                // ================================================
                driverController
                                .x()
                                .onTrue(
                                                Commands.runOnce(() -> SmartController.getInstance()
                                                                .setDriveMode(DriveModeType.SAFE))
                                                                .alongWith(
                                                                                Commands.runOnce(() -> SmartController
                                                                                                .getInstance()
                                                                                                .disableSmartControl())));

                // ================================================
                // DRIVER CONTROLLER - Y
                // SET DRIVE MODE TO FEED
                // ================================================
                driverController
                                .y()
                                .onTrue(
                                                Commands.runOnce(() -> SmartController.getInstance()
                                                                .setDriveMode(DriveModeType.FEED))
                                                                .alongWith(
                                                                                Commands.runOnce(() -> SmartController
                                                                                                .getInstance()
                                                                                                .enableSmartControl())));

                // ================================================
                // DRIVER CONTROLLER - A
                // SET DRIVE MODE TO AMP
                // ================================================
                driverController
                                .povDown()
                                .onTrue(
                                                Commands.runOnce(
                                                                () -> SmartController.getInstance()
                                                                                .setDriveMode(DriveModeType.GAMEPIECE))
                                                                .alongWith(
                                                                                Commands.runOnce(() -> SmartController
                                                                                                .getInstance()
                                                                                                .enableSmartControl())));

                // ================================================
                // DRIVER CONTROLLER - RIGHT TRIGGER
                // SCORE
                // ================================================
                driverController
                                .rightTrigger()
                                .whileTrue(new SmartShoot(arm, shooter, magazine, beamBreak, drive::getPose, 1.5));
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

                // ================================================
                // DEMO CONTROLLER - LEFT BUMPER
                // RUN INTAKE IN
                // ================================================
                demoController
                                .leftBumper()
                                .whileTrue(Commands.startEnd(intake::enableIntakeRequest,
                                                intake::disableIntakeRequest));

                // ================================================
                // DEMO CONTROLLER - LEFT TRIGGER
                // RUN INTAKE OUT
                // ================================================
                demoController
                                .leftTrigger()
                                .whileTrue(Commands.startEnd(intake::enableOuttakeRequest,
                                                intake::disableOuttakeRequest));

                // ================================================
                // DEMO CONTROLLER - RIGHT TRIGGER
                // SCORE
                // ================================================
                demoController
                                .rightTrigger()
                                .whileTrue(new ManualShoot(shooter, magazine, beamBreak, 0.5))
                                .onFalse(
                                                Commands.runOnce(() -> SmartController.getInstance()
                                                                .setDriveMode(DriveModeType.SAFE)));

                // ================================================
                // DEMO CONTROLLER - B
                // PREPARE SHOOT
                // ================================================
                demoController
                                .b()
                                .onTrue(
                                                Commands.runOnce(() -> SmartController.getInstance()
                                                                .setDriveMode(DriveModeType.DEMO))
                                                                .alongWith(
                                                                                Commands.runOnce(() -> SmartController
                                                                                                .getInstance()
                                                                                                .disableSmartControl())));

                // ================================================
                // DEMO CONTROLLER - X
                // SET DRIVE MODE TO SAFE
                // ================================================
                demoController
                                .x()
                                .onTrue(
                                                Commands.runOnce(() -> SmartController.getInstance()
                                                                .setDriveMode(DriveModeType.SAFE))
                                                                .alongWith(
                                                                                Commands.runOnce(() -> SmartController
                                                                                                .getInstance()
                                                                                                .disableSmartControl())));

                // ================================================
                // DEMO CONTROLLER - DPAD UP
                // MOVE CLIMBER UP
                // ================================================
                demoController.povUp().onTrue(new ManualClimb(climberLeft, climberRight, -140));

                // ================================================
                // DEMO CONTROLLER - DPAD DOWN
                // MOVE CLIMBER DOWN
                // ================================================
                demoController.povDown().onTrue(new ManualClimb(climberLeft, climberRight, 0));
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
