package frc.robot.subsystems;

import frc.robot.Constants.PullUpConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

//This is climbing
public class PullUpSubsystem extends SubsystemBase {

        private SparkMax m_PrimaryMotor = new SparkMax(PullUpConstants.kPullUpPrimaryMotorPort,
                        MotorType.kBrushless);
        private SparkMax m_SecondaryMotor = new SparkMax(PullUpConstants.kPullUpSecondaryMotorPort,
                        MotorType.kBrushless);
        private SparkMaxConfig primaryConfig = new SparkMaxConfig();
        private SparkMaxConfig secondaryConfig = new SparkMaxConfig();
        private SparkClosedLoopController m_PullUpPrimaryController = m_PrimaryMotor.getClosedLoopController();
        // private SparkClosedLoopController m_ElevatorSecondaryController =
        // m_SecondaryMotor.getClosedLoopController();

        private SparkLimitSwitch forwardLimitSwitch;
        private SparkLimitSwitch reverseLimitSwitch;
        private RelativeEncoder encoder;

        public PullUpSubsystem() {

                primaryConfig
                                .inverted(true)
                                .idleMode(IdleMode.kBrake);

                // secondaryConfig
                // .inverted(false)
                // .idleMode(IdleMode.kBrake);
                secondaryConfig.follow(m_PrimaryMotor.getDeviceId(), true);

                forwardLimitSwitch = m_PrimaryMotor.getForwardLimitSwitch();
                reverseLimitSwitch = m_PrimaryMotor.getReverseLimitSwitch();
                encoder = m_PrimaryMotor.getEncoder();

                primaryConfig.limitSwitch
                                .forwardLimitSwitchType(Type.kNormallyOpen)
                                .forwardLimitSwitchEnabled(true)
                                .reverseLimitSwitchType(Type.kNormallyOpen)
                                .reverseLimitSwitchEnabled(true);

                primaryConfig.softLimit
                                .forwardSoftLimit(PullUpConstants.kPullUpForwardSoftLimitRotations)
                                .forwardSoftLimitEnabled(true)
                                .reverseSoftLimit(PullUpConstants.kPullUpReverseSoftLimitRotations)
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

                m_PrimaryMotor.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                m_SecondaryMotor.configure(secondaryConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                encoder.setPosition(0);
                SmartDashboard.setDefaultBoolean("Pullp/direction", true);
                SmartDashboard.setDefaultNumber("PullUp/Target Position", 0);
                SmartDashboard.setDefaultNumber("PullUp/Target Velocity", 0);
                SmartDashboard.setDefaultBoolean("PullUp/Control Mode", false);
                SmartDashboard.setDefaultBoolean("PullUp/Reset Encoder", false);
        }

        public void setPullUpVelocity(double targetVelocity) {
                // double targetVelocity = SmartDashboard.getNumber("Elevator/Target Velocity",
                // 0);
                m_PullUpPrimaryController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        }

        public void setPullUpPosition(double targetPosition) {
                // double targetPosition = SmartDashboard.getNumber("Elevator/Target Position",
                // 0);
                m_PullUpPrimaryController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        @Override
        public void periodic() {
                // Display data from SPARK onto the dashboard
                SmartDashboard.putBoolean("PullUp/Forward Limit Reached", forwardLimitSwitch.isPressed());
                SmartDashboard.putBoolean("PullUp/Reverse Limit Reached", reverseLimitSwitch.isPressed());
                SmartDashboard.putNumber("PullUp/Applied Output", m_PrimaryMotor.getAppliedOutput());
                SmartDashboard.putNumber("PullUp/Position", encoder.getPosition());

                SmartDashboard.putNumber("PullUp/PrimaryMotor set output", m_PrimaryMotor.get());
                SmartDashboard.putNumber("PullUp/SecondaryMotor set output", m_SecondaryMotor.get());

                SmartDashboard.putNumber("PullUp/Actual Position", encoder.getPosition());
                SmartDashboard.putNumber("PullUp/Actual Velocity", encoder.getVelocity());

                if (SmartDashboard.getBoolean("PullUp/Reset Encoder", false)) {
                        SmartDashboard.putBoolean("PullUp/Reset Encoder", false);
                        // Reset the encoder position to 0
                        encoder.setPosition(0);
                }
        }

}