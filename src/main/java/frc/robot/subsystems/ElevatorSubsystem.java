package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax m_PrimaryMotor = new SparkMax(ElevatorConstants.kPrimaryElevatorMotorPort,
            MotorType.kBrushless);
    private SparkMax m_SecondaryMotor = new SparkMax(ElevatorConstants.kSecondaryElevatorMotorPort,
            MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();
    private SparkMaxConfig config2 = new SparkMaxConfig();
    private SparkClosedLoopController m_ElevatorPrimaryController = m_PrimaryMotor.getClosedLoopController();
    private SparkClosedLoopController m_ElevatorSecondaryController = m_SecondaryMotor.getClosedLoopController();

    public ElevatorSubsystem() {

        config
                .inverted(true)
                .idleMode(IdleMode.kBrake);

        config2
                .inverted(false)
                .idleMode(IdleMode.kBrake);

        m_PrimaryMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_SecondaryMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setElevatorSpeed(double speed) {
        m_PrimaryMotor.set(speed);
        m_SecondaryMotor.set(speed);
        writeMetricsToSmartDashboard();
    }

    public void setElevatorPosition(double rotations) {
        m_ElevatorPrimaryController.setReference(rotations, SparkMax.ControlType.kPosition);
        m_ElevatorSecondaryController.setReference(rotations, SparkMax.ControlType.kPosition);
        writeMetricsToSmartDashboard();
    }

    public void writeMetricsToSmartDashboard() {
        SmartDashboard.putNumber("Motor set output", m_PrimaryMotor.get());
        SmartDashboard.putNumber("Motor set output", m_SecondaryMotor.get());
    }

}