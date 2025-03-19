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
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {

    private SparkMax m_GrabberMotor = new SparkMax(GrabberConstants.kGrabberMotorPort,
            MotorType.kBrushless);
    private SparkMaxConfig primaryConfig = new SparkMaxConfig();
    private SparkClosedLoopController m_GrabberController = m_GrabberMotor.getClosedLoopController();

    private SparkLimitSwitch forwardLimitSwitch;
    private RelativeEncoder encoder;

    public GrabberSubsystem() {

        primaryConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake);

        forwardLimitSwitch = m_GrabberMotor.getForwardLimitSwitch();
        encoder = m_GrabberMotor.getEncoder();

        primaryConfig.limitSwitch
                .forwardLimitSwitchType(Type.kNormallyOpen)
                .forwardLimitSwitchEnabled(true);

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

        m_GrabberMotor.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0);
        SmartDashboard.setDefaultBoolean("Grabber/direction", true);
        SmartDashboard.setDefaultNumber("Grabber/Target Position", 0);
        SmartDashboard.setDefaultNumber("Grabber/Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Grabber/Control Mode", false);
        SmartDashboard.setDefaultBoolean("Grabber/Reset Encoder", false);
    }

    public void setGrabberVelocity(double targetVelocity) {
        m_GrabberController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        System.out.println("trying to set motor to speed " + targetVelocity);
    }

    public void setGrabberPosition(double targetPosition) {
        m_GrabberController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void periodic() {
        // Display data from SPARK onto the dashboard
        SmartDashboard.putBoolean("Grabber/Forward Limit Reached", forwardLimitSwitch.isPressed());
        SmartDashboard.putNumber("Grabber/Applied Output", m_GrabberMotor.getAppliedOutput());
        SmartDashboard.putNumber("Grabber/Position", encoder.getPosition());

        SmartDashboard.putNumber("Grabber/PrimaryMotor set output", m_GrabberMotor.get());

        SmartDashboard.putNumber("Grabber/Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator/Actual Velocity", encoder.getVelocity());

        if (SmartDashboard.getBoolean("Grabber/Reset Encoder", false)) {
            SmartDashboard.putBoolean("Grabber/Reset Encoder", false);
            // Reset the encoder position to 0
            encoder.setPosition(0);
        }
    }

}