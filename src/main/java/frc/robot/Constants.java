// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ElevatorConstants {
    public static final int kPrimaryElevatorMotorPort = 31;
    public static final int kSecondaryElevatorMotorPort = 32;
    public static final int kForwardSoftLimitRotations = 50;
    public static final int kReverseSoftLimitRotations = -50;

  }

  public static class ArmJointsConstants {
    public static final int kArmMotorPort = 34;
    public static final int kArmForwardSoftLimitRotations = 15;
    public static final int kArmReverseSoftLimitRotations = -15;
  }

  public static class GrabberConstants {
    public static final int kGrabberMotorPort = 33;

  }

  public static class AlgaeConstants {
    public static final int kAlgaePrimaryMotorPort = 0;
    public static final int kAlgaeSecondaryMotorPort = 0;
    public static final int kAlgaeForwardSoftLimitRotations = 0;
    public static final int kAlgaeReverseSoftLimitRotations = 0;
  }

  public static class PullUpConstants {
    public static final int kPullUpPrimaryMotorPort = 0;
    public static final int kPullUpSecondaryMotorPort = 0;
    public static final int kPullUpForwardSoftLimitRotations = 50;
    public static final int kPullUpReverseSoftLimitRotations = -50;
  }
}
