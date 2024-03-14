// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.MultiDistanceShot;
import frc.robot.commands.PathFinderAndFollow;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOKrakenFOC;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.DriveController.DriveModeType;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersSensorsIO;
import frc.robot.subsystems.rollers.RollersSensorsIOReal;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIO;
import frc.robot.subsystems.rollers.intake.IntakeIOKrakenFOC;
import frc.robot.subsystems.rollers.intake.IntakeIOSim;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVisionSIM;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private static DriveController driveMode = new DriveController();
    private AprilTagVision aprilTagVision;
    private final Flywheel flywheel;
    private final Rollers rollers;
    private final Arm arm;

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    // private final LoggedTunableNumber flywheelSpeedInput =
    // new LoggedTunableNumber("Flywheel Speed", 1500.0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Declare component subsystems (not visible outside constructor)
        Intake intake = null;
        // Climber climber = null;
        RollersSensorsIO rollersSensorsIO = null;
        switch (Constants.getMode()) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(true),
                        new ModuleIOTalonFX(moduleConfigs[0]),
                        new ModuleIOTalonFX(moduleConfigs[1]),
                        new ModuleIOTalonFX(moduleConfigs[2]),
                        new ModuleIOTalonFX(moduleConfigs[3]));
                aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight("limelight"));
                flywheel = new Flywheel(new FlywheelIOTalonFX());
                intake = new Intake(new IntakeIOKrakenFOC());
                rollersSensorsIO = new RollersSensorsIOReal();
                rollers = new Rollers(intake, rollersSensorsIO);
                arm = new Arm(new ArmIOKrakenFOC());
                // climber = new Climber(new ClimberIOKrakenFOC());

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
                aprilTagVision = new AprilTagVision(
                        new AprilTagVisionIOPhotonVisionSIM(
                                "photonCamera1",
                                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)),
                                drive::getDrive));
                flywheel = new Flywheel(new FlywheelIOSim());
                intake = new Intake(new IntakeIOSim());
                rollersSensorsIO = new RollersSensorsIO() {
                };
                rollers = new Rollers(intake, rollersSensorsIO);

                arm = new Arm(new ArmIOSim());
                // climber = new Climber(new ClimberIOSim());

                break;

            default:
                // Replayed robot, disable IO implementations
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
                flywheel = new Flywheel(new FlywheelIO() {
                });

                aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {
                });

                intake = new Intake(new IntakeIO() {
                });

                arm = new Arm(new ArmIO() {
                });

                rollers = new Rollers(intake, rollersSensorsIO);
        }

        // Set up auto routines
        // NamedCommands.registerCommand(
        // "Run Flywheel",
        // Commands.startEnd(
        // () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
        // flywheel)
        // .withTimeout(5.0));
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
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        driveMode,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        driverController.rightBumper()
                .whileTrue(Commands.runOnce(() -> driveMode.setDriveMode(DriveModeType.AMP)).alongWith(
                        Commands.startEnd(
                                () -> driveMode.enableHeadingControl(),
                                () -> driveMode.disableHeadingControl())));

        driverController.povLeft().whileTrue(new PathFinderAndFollow(driveMode.getDriveModeType()));

        driverController
                .leftTrigger()
                .whileTrue(
                        Commands.run(() -> driveMode.setDriveMode(DriveModeType.SPEAKER)).alongWith(
                                Commands.startEnd(
                                        () -> driveMode.enableHeadingControl(),
                                        () -> driveMode.disableHeadingControl())));

        driverController
                .y()
                .whileTrue(
                        Commands.runOnce(
                                () -> drive.setAutoStartPose(
                                        new Pose2d(new Translation2d(4, 5), Rotation2d.fromDegrees(0)))));
        driverController
                .povDown()
                .whileTrue(
                        new DriveToPoint(
                                drive, new Pose2d(new Translation2d(2.954, 3.621), Rotation2d.fromRadians(2.617))));

        driverController
                .povUp()
                .whileTrue(
                        new MultiDistanceShot(
                                drive::getPose,
                                FieldConstants.Speaker.centerSpeakerOpening.getTranslation(),
                                flywheel));

        driverController
                .povUp()
                .whileTrue(
                        Commands.runOnce(
                                () -> arm.setDesiredDegrees(90)));

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
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
