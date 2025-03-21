
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmJointsSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LightingSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));

  }

  public static Command lightStrobe(LightingSubsystem subsystem) {
    return Commands.run(() -> subsystem.pinkWhiteStrobe());

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public class AutoRoutines {
    private ElevatorSubsystem m_elevatorsSubsystem;
    private ArmJointsSubsystem m_ArmJointsSubsystem;
    private GrabberSubsystem m_GrabberSubsystem;

    public static Command MoveCoralToLs(ElevatorSubsystem m_elevatorSubsystem,
        ArmJointsSubsystem m_ArmJointsSubsystem, GrabberSubsystem m_GrabberSubsystem, int ElevatorPosition,
        int Armposition) {
      return Commands.sequence(
          Commands.sequence(
              Commands.runOnce(() -> m_ArmJointsSubsystem.setArmJointPosition(0)),
              Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(0)),
              Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorPosition)),
              Commands.runOnce(() -> m_ArmJointsSubsystem.setArmJointPosition(Armposition)),
              Commands.runOnce(() -> m_GrabberSubsystem.setGrabberVelocity(5000)),
              Commands.runOnce(() -> m_ArmJointsSubsystem.setArmJointPosition(0)),
              Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(0))).withTimeout(2));
    }

    // public static Command GrabOrDropCoralT() {
    // return Commands.sequence(
    // Commands.sequence(
    // MoveCoralToLs(null, null, 1, 1)).withTimeout(2));

    // }
  }
}
