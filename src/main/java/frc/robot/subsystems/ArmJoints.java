package frc.robot.subsystems;

import frc.robot.Constants.ArmJointsConstants;
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

public class ArmJoints extends SubsystemBase {
    private SparkMax m_ArmMotor = new SparkMax(ArmJointsConstants.kArmMotorPort, MotorType.kBrushless);
    // private RelativeEncoder m_armEncoder = new m_ArmMotor.getEncoder();
}
