package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmJointsSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class MoveCoralToLs extends Command {
    private final ElevatorSubsystem m_Elevator;
    private final ArmJointsSubsystem m_ArmJoint;
    private final GrabberSubsystem m_Grabber;
    private final int ePose;
    private final int aPose;

    public MoveCoralToLs(ElevatorSubsystem elevator, ArmJointsSubsystem armJoint, GrabberSubsystem grabber,
            int elevatorLocation, int armJointLocation) {
        m_Elevator = elevator;
        m_ArmJoint = armJoint;
        m_Grabber = grabber;
        ePose = elevatorLocation;
        aPose = armJointLocation;
        addRequirements(m_Elevator);
        addRequirements(m_ArmJoint);
        addRequirements(m_Grabber);

    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_Elevator.setElevatorPosition(ePose);
        Timer.delay(0.5);
        m_ArmJoint.setArmJointPosition(aPose);
        Timer.delay(1);
        m_Grabber.setGrabberVelocity(5000);
        Timer.delay(1.5);
        m_Grabber.setGrabberVelocity(0);
        m_ArmJoint.setArmJointPosition(0);
        m_Elevator.setElevatorPosition(0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // end the command if we have run for a specific amount of time
        return true;

    }
}