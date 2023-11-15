// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static double deadband = 0.1;

  public static int steerFrontLeft = 10;
  public static int driveFrontLeft = 11;
  public static int frontLeftCANCoder = 12;

  public static int steerFrontRight = 13;
  public static int driveFrontRight = 14;
  public static int frontRightCANCoder = 15;

  public static int steerRearLeft = 16;
  public static int driveRearLeft = 17;
  public static int rearLeftCANCoder = 18;

  public static int steerRearRight = 19;
  public static int driveRearRight = 20;
  public static int rearRightCANCoder = 21;

  public static int elevator = 31;
  public static int wrist = 32;
  public static int vertRoller = 33;
  public static int horiRoller = 34;

  public static int driveMotorCurrentLimit = 55;
  public static int steerMotorCurrentLimit = 45;
  public static int hRollerCurrentLimit = 15;
  public static int vertRollerCurrentLimit = 80;

  public static int intakeSolenoid = 0;

  public static int beamSensor = 0;

  public static int driverController = 0;
  public static int operatorController = 1;

  public static int upperLimit = 1;
  public static int lowerLimit = 2;

  public static double elevator_kP = 24;
  public static double elevator_kD = 0.1;

  public static double wrist_kP = 24;
  public static double wrist_kD = 5;

  public static double vertRoller_kP = 0.1;
  public static double vertRoller_kD = 0.1;

  public static double wheelDiameter = 4.0;
  public static double wheelCircumference = wheelDiameter * Math.PI;

  // Positions for the elevator
  public static double elevatorScoreConeBottom =  3.0;
  public static double elevatorScoreConeMiddle = 136.83;

  public static double elevatorScoreCubeBottom = 3.0;
  public static double elevatorScoreCubeMiddle = 137.83;
  public static double elevatorScoreCubeTop = 0.0;

  public static double elevatorGroundCone = 0.8;
  public static double elevatorGroundCube = 53.8;
  public static double elevatorDoublePickup = 0.0;
  public static double elevatorSinglePickup = 0.0;

  public static double elevatorStow = 2.0;
  
  // Wrist
  public static double wristScoreConeBottom =  15.7;
  public static double wristScoreConeMiddle = 12.4;

  public static double wristScoreCubeBottom = 15.7;
  public static double wristScoreCubeMiddle = 14.1;
  public static double wristScoreCubeTop = 0.0;

  public static double wristGroundCone = 15.7;
  public static double wristGroundCube = 22.4;
  public static double wristDoublePickup = 0.0;
  public static double wristSinglePickup = 0.0;

  public static double wristTravel = 0.0;
  public static double wristStow = 0.75;
}