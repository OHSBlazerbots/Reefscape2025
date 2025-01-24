package frc.robot.subsystems;

import frc.robot.Constants.ArmJointsConstants;
//import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmJointsSubsytem extends SubsystemBase {

    // private static final SparkPIDController m_shooterController = new
    // SparkPIDController(
    // ShooterConstants.kSho"1oterGains,
    // ShooteIrConstants.kShooterMotorPort);
    private SparkMax m_armMotor = new SparkMax(ArmJointsConstants.kArmMotorPort, MotorType.kBrushless);
    private RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
    private SparkClosedLoopController m_armController = m_armMotor.getClosedLoopController();
    private SparkMaxConfig config = new SparkMaxConfig();
    private AbsoluteEncoder m_armjointAbsoluteEncoder = m_armMotor.getAbsoluteEncoder();
    // public motorController swivel = new motorController(m_SwivelController, 0.15,
    // 0, 0, 0, 1, 1, -1, 5700);

    public ArmJointsSubsytem() {
        config
                .idleMode(IdleMode.kBrake);

        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.15)
                .i(0)
                .d(0);

        m_armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setArmSpeed(double speed) {
        // speed = SmartDashboard.getNumber("Shooter/Speed Output", 0);
        // System.out.println("speed=" + speed);
        // System.out.println("dashboard=" + SmartDashboard.getNumber("Shooter/Speed
        // Output", 0));
        m_armMotor.set(speed);
    }

    // public void setSwivelPosition(double rotations){
    // m_SwivelController.setReference(rotations,
    // CANSparkMax.ControlType.kPosition);
    // writeMetricsToSmartDashboard();
    // }

    // public void writeMetricsToSmartDashboard() {
    // intake.writeMetricsToSmartDashboard();
    // SmartDashboard.putNumber("Motor set output", m_intakeMotor.get());
    // SmartDashboard.putNumber("Motor set output", m_swivelMotor.get());
    // SmartDashboard.putNumber("ProcessVariable", m_swivelEncoder.getPosition());
    // }

}
