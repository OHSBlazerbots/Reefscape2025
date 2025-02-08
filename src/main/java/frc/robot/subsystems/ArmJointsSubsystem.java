package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmJointsConstants;

public class ArmJointsSubsystem extends SubsystemBase {

    private SparkMax m_armMotor = new SparkMax(ArmJointsConstants.kArmMotorPort, MotorType.kBrushless);
    private SparkMaxConfig primaryConfig = new SparkMaxConfig();
    private SparkClosedLoopController m_armJointController = m_armMotor.getClosedLoopController();

    private SparkLimitSwitch forwardLimitSwitch;
    private SparkLimitSwitch reverseLimitSwitch;
    private RelativeEncoder encoder;

    public ArmJointsSubsystem() {

        primaryConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);

        forwardLimitSwitch = m_armMotor.getForwardLimitSwitch();
        reverseLimitSwitch = m_armMotor.getReverseLimitSwitch();
        encoder = m_armMotor.getEncoder();

        primaryConfig.limitSwitch
                .forwardLimitSwitchType(Type.kNormallyOpen)
                .forwardLimitSwitchEnabled(true)
                .reverseLimitSwitchType(Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true);

        primaryConfig.softLimit
                .forwardSoftLimit(ArmJointsConstants.kArmForwardSoftLimitRotations)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ArmJointsConstants.kArmReverseSoftLimitRotations)
                .reverseSoftLimitEnabled(true);

        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */
        primaryConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);

        /*
         * Configure the closed loop controller. We want to make sure we set the
         * feedback sensor as the primary encoder.
         */
        primaryConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        m_armMotor.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0);
        SmartDashboard.setDefaultBoolean("ArmJoints/direction", true);
        SmartDashboard.setDefaultNumber("ArmJoints/Target Position", 0);
        SmartDashboard.setDefaultNumber("ArmJoints/Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("ArmJoints/Control Mode", false);
        SmartDashboard.setDefaultBoolean("ArmJoints/Reset Encoder", false);
    }

    public void setArmJointVelocity(double targetVelocity) {
        m_armJointController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    public void setArmJointPosition(double targetPosition) {
        m_armJointController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void periodic() {
        // Display data from SPARK onto the dashboard
        SmartDashboard.putBoolean("ArmJoint/Forward Limit Reached", forwardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("ArmJoint/Reverse Limit Reached", reverseLimitSwitch.isPressed());
        SmartDashboard.putNumber("ArmJoint/Applied Output", m_armMotor.getAppliedOutput());
        SmartDashboard.putNumber("ArmJoint/Position", encoder.getPosition());

        SmartDashboard.putNumber("ArmJoint/PrimaryMotor set output", m_armMotor.get());

        SmartDashboard.putNumber("ArmJoint/Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("ArmJoint/Actual Velocity", encoder.getVelocity());

        if (SmartDashboard.getBoolean("ArmJoint/Reset Encoder", false)) {
            SmartDashboard.putBoolean("ArmJoint/Reset Encoder", false);
            // Reset the encoder position to 0
            encoder.setPosition(0);
        }
    }

}
