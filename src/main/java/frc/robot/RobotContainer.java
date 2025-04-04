// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmJointsConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.MoveCoralToLs;
import frc.robot.subsystems.ArmJointsSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriverCameraSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.PullUpSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        private final LightingSubsystem m_LightingSubsystem;
        private final ElevatorSubsystem m_elevatorSubsystem;
        private final ArmJointsSubsystem m_ArmJointsSubsystem;
        private final GrabberSubsystem m_GrabberSubsystem;
        private final PullUpSubsystem m_PullUpSubsystem;

        private final DriverCameraSubsystem m_DriverCameraSubsystem;
        private final DriveSubsystem drivebase;

        private final CommandXboxController m_DrivController;
        private final CommandXboxController m_CodrivController;

        SendableChooser<Command> m_chooser = new SendableChooser<>();
        private Command m_moveToL4;
        private Command m_moveToL3;
        private Command FirstAuto;
        private Command StraightAuto;
        private Command Middle;
        private Command RedEdge;
        private Command BlueEdge;
        private Command SafetyAuto;
        SwerveInputStream driveAngularVelocity;
        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle;
        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        SwerveInputStream driveRobotOriented;

        SwerveInputStream driveAngularVelocityKeyboard;
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                m_LightingSubsystem = new LightingSubsystem();
                m_elevatorSubsystem = new ElevatorSubsystem();
                m_ArmJointsSubsystem = new ArmJointsSubsystem();
                m_GrabberSubsystem = new GrabberSubsystem();
                m_PullUpSubsystem = new PullUpSubsystem();
                m_DriverCameraSubsystem = new DriverCameraSubsystem();
                drivebase = new DriveSubsystem(new File(Filesystem.getDeployDirectory(),
                                "swerve/neo"));

                m_DrivController = new CommandXboxController(0);
                m_CodrivController = new CommandXboxController(1);

                m_chooser.addOption("Default", StraightAuto);
                m_chooser.addOption("ToReeftoStation", FirstAuto);
                m_chooser.addOption("SafetyAuto", SafetyAuto);
                m_chooser.addOption("Middle", Middle);
                m_chooser.addOption("RedEdge", RedEdge);
                m_chooser.addOption("BlueEdge", BlueEdge);

                SmartDashboard.putData(m_chooser);

                driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                () -> m_DrivController.getLeftY() * -1,
                                () -> m_DrivController.getLeftX() * -1)
                                .withControllerRotationAxis(m_DrivController::getRightX)
                                .deadband(0.01) // replace value later
                                .scaleTranslation(0.8)
                                .allianceRelativeControl(true);

                /**
                 * Clone's the angular velocity input stream and converts it to a fieldRelative
                 * input stream.
                 */
                driveDirectAngle = driveAngularVelocity.copy()
                                .withControllerHeadingAxis(m_DrivController::getRightX,
                                                m_DrivController::getRightY)
                                .headingWhile(true);

                /**
                 * Clone's the angular velocity input stream and converts it to a robotRelative
                 * input stream.
                 */
                driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                .allianceRelativeControl(false);

                driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                () -> -m_DrivController.getLeftY(),
                                () -> -m_DrivController.getLeftX())
                                .withControllerRotationAxis(() -> m_DrivController.getRawAxis(
                                                2))
                                .deadband(0.01)// replace value later
                                .scaleTranslation(0.8)
                                .allianceRelativeControl(true);
                // Derive the heading axis with math!
                driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                .withControllerHeadingAxis(() -> Math.sin(
                                                m_DrivController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                                () -> Math.cos(
                                                                m_DrivController.getRawAxis(2) * Math.PI)
                                                                * (Math.PI * 2))
                                .headingWhile(true);

                SendableChooser<Command> m_chooser = new SendableChooser<>();

                m_moveToL4 = new MoveCoralToLs(m_elevatorSubsystem, m_ArmJointsSubsystem, m_GrabberSubsystem,
                                ElevatorConstants.maxElevatorHeight, ArmJointsConstants.l4ArmAngleAuto);

                m_moveToL3 = new MoveCoralToLs(m_elevatorSubsystem, m_ArmJointsSubsystem, m_GrabberSubsystem,
                                0, 75);

                NamedCommands.registerCommand("L4Score", m_moveToL4);
                NamedCommands.registerCommand("L3Score", m_moveToL3);
                FirstAuto = drivebase.getAutonomousCommand("FirstAuto");
                StraightAuto = drivebase.getAutonomousCommand("drive straight");
                RedEdge = drivebase.getAutonomousCommand("RedEdge");
                BlueEdge = drivebase.getAutonomousCommand("BlueEdge");
                Middle = drivebase.getAutonomousCommand("Middle");
                SafetyAuto = drivebase.getAutonomousCommand("SafetyAuto");
                configureBindings();

        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button. CommandJoystick Flight
         * joysticks}.
         */

        private void configureBindings() {
                Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
                Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngle);
                Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
                Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                                .driveFieldOriented(driveAngularVelocityKeyboard);
                Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngleKeyboard);

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }

                if (Robot.isSimulation()) {
                        m_DrivController.start()
                                        .onTrue(Commands.runOnce(() -> drivebase
                                                        .resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
                        m_DrivController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

                }
                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command
                                                                                 // above!

                m_DrivController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                m_DrivController.rightBumper().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
                m_DrivController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
                m_DrivController.povLeft().whileTrue(getAutonomousCommand());

                m_DrivController.back().whileTrue(drivebase.centerModulesCommand());
                m_CodrivController
                                .y()
                                .onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(78)))
                                .onFalse(Commands.none());

                m_CodrivController
                                .a()
                                .onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(0)))
                                .onFalse(Commands.none());

                m_CodrivController
                                .b()
                                .onTrue(Commands.runOnce(() -> m_ArmJointsSubsystem.setArmJointPosition(75)))
                                .onFalse(Commands.none());

                m_CodrivController
                                .x()
                                .onTrue(Commands.runOnce(() -> m_ArmJointsSubsystem.setArmJointPosition(0)))
                                .onFalse(Commands.none());

                m_CodrivController
                                .rightBumper()
                                .onTrue(Commands.runOnce(() -> m_GrabberSubsystem.setGrabberVelocity(-5000)))
                                .onFalse(Commands.runOnce(() -> m_GrabberSubsystem.setGrabberVelocity(0)));

                m_CodrivController
                                .leftBumper()
                                .onTrue(Commands.runOnce(() -> m_GrabberSubsystem.setGrabberVelocity(5000)))
                                .onFalse(Commands.runOnce(() -> m_GrabberSubsystem.setGrabberVelocity(0)));

                m_CodrivController
                                .povUp()
                                .onTrue(Commands.runOnce(() -> m_ArmJointsSubsystem
                                                .setArmJointPosition(ArmJointsConstants.l4ArmAngleTeleop)))
                                .onFalse(Commands.none());

                m_CodrivController
                                .povDown()
                                .onTrue(m_moveToL4)
                                .onFalse(Commands.none());

                m_CodrivController
                                .povRight()
                                .onTrue(Commands.runOnce(() -> m_ArmJointsSubsystem.setArmJointVelocity(2000)))
                                .onFalse(Commands.runOnce(() -> m_ArmJointsSubsystem.setArmJointVelocity(0)));

                m_CodrivController
                                .povLeft()
                                .onTrue(Commands.runOnce(() -> m_ArmJointsSubsystem.setArmJointVelocity(-2000)))
                                .onFalse(Commands.runOnce(() -> m_ArmJointsSubsystem.setArmJointVelocity(0)));

                m_CodrivController
                                .start()
                                .onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setElevatorVelocity(1000)))
                                .onFalse(Commands.runOnce(() -> m_elevatorSubsystem.setElevatorVelocity(0)));

                m_CodrivController
                                .back()
                                .onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setElevatorVelocity(-1000)))
                                .onFalse(Commands.runOnce(() -> m_elevatorSubsystem.setElevatorVelocity(0)));

                m_CodrivController
                                .rightTrigger()
                                .onTrue(Commands.runOnce(() -> m_PullUpSubsystem.setPullUpPosition(100)))
                                .onFalse(Commands.runOnce(() -> m_PullUpSubsystem.setPullUpVelocity(0)));

                m_CodrivController
                                .leftTrigger()
                                .onTrue(Commands.runOnce(() -> m_PullUpSubsystem.setPullUpPosition(-100)))
                                .onFalse(Commands.runOnce(() -> m_PullUpSubsystem.setPullUpVelocity(0)));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                // return m_chooser.getSelected();
                return Middle;
                // return FirstAuto;

                // Autos.exampleAuto(m_elevatorSubsystem);
                // return drivebase.getAutonomousCommand("New Auto");
        }
}
