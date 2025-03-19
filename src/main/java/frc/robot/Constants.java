// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class DriveConstants {
    public static final int kTurnTravelUnitsPerRotation = 3600;
    public static final int kEncoderUnitsPerRotation = 51711; // number is added by experimentation
    public final static double kNeutralDeadband = 0.001;
    public final static int kTimeoutMs = 30;
    public final static int PID_PRIMARY = 0;
    public final static int REMOTE_0 = 0;
    public final static int REMOTE_1 = 1;
    public final static int PID_TURN = 1;
    public final static int SLOT_1 = 1;
    public final static int kSlot_Turning = SLOT_1;
  }

  public static class ElevatorConstants {
    public static final int kPrimaryElevatorMotorPort = 9;
    public static final int kSecondaryElevatorMotorPort = 10;
    public static final int kForwardSoftLimitRotations = 47;
    public static final int kReverseSoftLimitRotations = 0;

  }

  public static class ArmJointsConstants {
    public static final int kArmMotorPort = 11;
    public static final int kArmForwardSoftLimitRotations = 75;
    public static final int kArmReverseSoftLimitRotations = 0;
  }

  public static class GrabberConstants {
    public static final int kGrabberMotorPort = 12;

  }

  public static class AlgaeConstants {
    public static final int kAlgaePrimaryMotorPort = 69;
    public static final int kAlgaeSecondaryMotorPort = 70;
    public static final int kAlgaeForwardSoftLimitRotations = 0;
    public static final int kAlgaeReverseSoftLimitRotations = 0;
    public static final int kAlgaeUpSoftLimit = 10;
    public static final int kAlgaeDownSoftLimit = 0;
  }

  public static class PullUpConstants {
    public static final int kPullUpPrimaryMotorPort = 80;
    public static final int kPullUpSecondaryMotorPort = 90;
    public static final int kPullUpForwardSoftLimitRotations = 50;
    public static final int kPullUpReverseSoftLimitRotations = -50;
  }

 
}
