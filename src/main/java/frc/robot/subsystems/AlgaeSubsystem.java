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
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {

        private SparkMax IntakeMotor = new SparkMax(AlgaeConstants.kAlgaePrimaryMotorPort,
                        MotorType.kBrushless); // motor for wheels on grabber
        private SparkMax m_SwivelMotor = new SparkMax(AlgaeConstants.kAlgaeSecondaryMotorPort,
                        MotorType.kBrushless); // motor for swivel
        private SparkMaxConfig IntakeConfig = new SparkMaxConfig(); // for wheels on grabber
        private SparkMaxConfig SwivelConfig = new SparkMaxConfig(); // for swivel
        private SparkClosedLoopController m_IntakeController = IntakeMotor.getClosedLoopController();
        private SparkClosedLoopController m_SwivelController = m_SwivelMotor.getClosedLoopController();

        private SparkLimitSwitch intakeForwardLimitSwitch;
        private SparkLimitSwitch intakeReverseLimitSwitch;
        private SparkLimitSwitch swivelForwardLimitSwitch;
        private SparkLimitSwitch swivelReverseLimitSwitch;
        private RelativeEncoder intakeEncoder;
        private RelativeEncoder swivelEncoder;

        public AlgaeSubsystem() {

                IntakeConfig
                                .inverted(true)
                                .idleMode(IdleMode.kBrake);

                intakeForwardLimitSwitch = IntakeMotor.getForwardLimitSwitch();
                intakeReverseLimitSwitch = IntakeMotor.getReverseLimitSwitch();
                intakeEncoder = IntakeMotor.getEncoder();
                IntakeConfig
                                .inverted(true)
                                .idleMode(IdleMode.kBrake);

                IntakeConfig.limitSwitch
                                .forwardLimitSwitchType(Type.kNormallyOpen)
                                .forwardLimitSwitchEnabled(true)
                                .reverseLimitSwitchType(Type.kNormallyOpen)
                                .reverseLimitSwitchEnabled(true);

                IntakeConfig.softLimit
                                .forwardSoftLimit(AlgaeConstants.kAlgaeForwardSoftLimitRotations)
                                .forwardSoftLimitEnabled(true)
                                .reverseSoftLimit(AlgaeConstants.kAlgaeReverseSoftLimitRotations)
                                .reverseSoftLimitEnabled(true);

                /*
                 * Configure the encoder. For this specific example, we are using the
                 * integrated encoder of the NEO, and we don't need to configure it. If
                 * needed, we can adjust values like the position or velocity conversion
                 * factors.
                 */
                IntakeConfig.encoder
                                .positionConversionFactor(1)
                                .velocityConversionFactor(1);

                /*
                 * Configure the closed loop controller. We want to make sure we set the
                 * feedback sensor as the primary encoder.
                 */
                IntakeConfig.closedLoop
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

                IntakeMotor.configure(IntakeConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                SwivelConfig
                                .idleMode(IdleMode.kBrake);

                swivelForwardLimitSwitch = m_SwivelMotor.getForwardLimitSwitch();
                swivelReverseLimitSwitch = m_SwivelMotor.getReverseLimitSwitch();
                swivelEncoder = m_SwivelMotor.getEncoder();

                SwivelConfig.limitSwitch
                                .forwardLimitSwitchType(Type.kNormallyOpen)
                                .forwardLimitSwitchEnabled(true)
                                .reverseLimitSwitchType(Type.kNormallyOpen)
                                .reverseLimitSwitchEnabled(true);

                SwivelConfig.softLimit
                                .forwardSoftLimit(AlgaeConstants.kAlgaeUpSoftLimit)
                                .forwardSoftLimitEnabled(true)
                                .reverseSoftLimit(AlgaeConstants.kAlgaeDownSoftLimit)
                                .reverseSoftLimitEnabled(true);
                SwivelConfig.encoder
                                .positionConversionFactor(1)
                                .velocityConversionFactor(1);
                SwivelConfig.closedLoop
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
                m_SwivelMotor.configure(SwivelConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                intakeEncoder.setPosition(0);
                swivelEncoder.setPosition(0);
                SmartDashboard.setDefaultBoolean("Algae/direction", true);
                SmartDashboard.setDefaultNumber("Algae/Target Position", 0);
                SmartDashboard.setDefaultNumber("Algae/Target Velocity", 0);
                SmartDashboard.setDefaultBoolean("Algae/Control Mode", false);
                SmartDashboard.setDefaultBoolean("Algae/Reset Encoder", false);
        }

        public void setSwivelVelocity(double targetVelocity) {
                m_SwivelController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        }

        public void setIntakeVelocity(double targetVelocity) {
                m_IntakeController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        }

        public void setSwivelPosition(double targetPosition) {
                m_SwivelController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        public void setIntakePosition(double targetPosition) {
                m_IntakeController.setReference(targetPosition, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }

        @Override
        public void periodic() {
                // Display data from SPARK onto the dashboard
                SmartDashboard.putBoolean("Algae/Intake Forward Limit Reached", intakeForwardLimitSwitch.isPressed());
                SmartDashboard.putBoolean("Algae/Intake Reverse Limit Reached", intakeReverseLimitSwitch.isPressed());
                SmartDashboard.putBoolean("Algae/Swivel Forward Limit Reached", swivelForwardLimitSwitch.isPressed());
                SmartDashboard.putBoolean("Algae/Swivel Reverse Limit Reached", swivelReverseLimitSwitch.isPressed());
                SmartDashboard.putNumber("Algae/Intake Applied Output", IntakeMotor.getAppliedOutput());
                SmartDashboard.putNumber("Algae/Intake Position", intakeEncoder.getPosition());
                SmartDashboard.putNumber("Algae/Swivel Applied Output", m_SwivelMotor.getAppliedOutput());
                SmartDashboard.putNumber("Algae/Swivel Position", swivelEncoder.getPosition());

                SmartDashboard.putNumber("Algae/Intake Motor set output", IntakeMotor.get());
                SmartDashboard.putNumber("Algae/Swivel Motor set output", m_SwivelMotor.get());

                SmartDashboard.putNumber("Algae/Intake Position", intakeEncoder.getPosition());
                SmartDashboard.putNumber("Algae/Intake Velocity", intakeEncoder.getVelocity());
                SmartDashboard.putNumber("Algae/Swivel Position", swivelEncoder.getPosition());
                SmartDashboard.putNumber("Algae/Swivel Velocity", swivelEncoder.getVelocity());

                if (SmartDashboard.getBoolean("Algae/Intake Reset Encoder", false)) {
                        SmartDashboard.putBoolean("Algae/Intake Reset Encoder", false);
                        // Reset the encoder position to 0
                        intakeEncoder.setPosition(0);
                }

                if (SmartDashboard.getBoolean("Algae/Swivel Reset Encoder", false)) {
                        SmartDashboard.putBoolean("Algae/Swivel Reset Encoder", false);
                        // Reset the encoder position to 0
                        swivelEncoder.setPosition(0);
                }
        }

}
