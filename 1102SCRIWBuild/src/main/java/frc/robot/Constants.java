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
  }
    public static final int steerFrontLeft = 10;
    public static final int steerFrontRight = 11; 
    public static final int steerBackLeft = 12;
    public static final int steerBackRight = 13;
    public static int steerMotorCurrentLimit = 45;

    public static final int driveFrontLeft = 14;
    public static final int driveFrontRight = 15;
    public static final int driveBackRight = 16;
    public static final int driveBackLeft = 17; 
    public static int driveMotorCurrentLimit = 55;

    public static final int frontLeftCANcoder = 18;
    public static final int frontRightCANcoder = 19;
    public static final int backLeftCANcoder = 20;
    public static final int backRightCANcoder = 21;
   
     public static int elevator = 31;
    public static int wrist = 32;
    public static int vertroller= 33;
    public static int hroller = 34;
    public static int intakeSolenoid = 0;
    public static int driverController = 0;
    public static int operatorController = 1;
    public static double wheelDiameter = 4.0;
    public static double wheelCircumfrance = wheelDiameter * Math.PI;

    public static double elevator_KP = 0.1;
    public static double wrist_KP = 0.1;
    public static double vertRollerKP = 0.1;

    public static double bottomConePosition = 1.5;
    public static double middleConePosition = 1.6; 
    public static double bottomCubePosition = 1.7;
    public static double middleCubePosition = 1.8;
    public static double topCubePosition = 1.9; 
    public static double pickupCone = 2.0;
    public static double pickupCube = 2.1;

    public static double doubleSubPosition =2.2;
    public static double singleSubPosition = 2.3; 
    



    

    



    
   



  }


