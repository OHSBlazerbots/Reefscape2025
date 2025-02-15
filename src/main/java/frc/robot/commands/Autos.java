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

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public class AutoRoutines {
    private ElevatorSubsystem m_elevatorsSubsystem;
    private ArmJointsSubsystem m_ArmJointsSubsystem;
    public static Command MoveCoralToLs(ElevatorSubsystem m_elevatorSubsystem,
        ArmJointsSubsystem m_ArmJointsSubsystem, int ElevatorPosition, int Armposition) {
      return Commands.sequence(
          Commands.sequence(
              Commands.runOnce(() -> m_ArmJointsSubsystem.setArmJointPosition(0)),
              Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(0)),
              Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorPosition)),
              Commands.runOnce(() -> m_ArmJointsSubsystem.setArmJointPosition(Armposition))).withTimeout(2));
    }

    public static Command GrabOrDropCoralT() {
      return Commands.sequence(
          Commands.sequence(
              MoveCoralToLs(null, null, 1, 1)).withTimeout(2));

    }
  }
}
