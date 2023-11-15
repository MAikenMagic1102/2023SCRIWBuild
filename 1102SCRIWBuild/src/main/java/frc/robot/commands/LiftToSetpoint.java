// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;

public class LiftToSetpoint extends CommandBase {
  /** Creates a new ArmToNode. */
  private Lift m_Lift;

  private double wristSetpoint = 0.0;
  private double elevatorSetpoint = 0.0;

  private boolean isFirstRun = true;
  private boolean isFinished = false;

  int i = 0;

  public LiftToSetpoint(Lift lift, double wristSetpoint, double elevatorSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Lift = lift;
    this.wristSetpoint = elevatorSetpoint;
    this.elevatorSetpoint = wristSetpoint;
    addRequirements(m_Lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("init");
    //System.out.println("Upper Setpoint" + upperArmSetpoint);
    //System.out.println("Lower Setpoint" + lowerArmSetpoint);
    isFirstRun = true;
    //System.out.println("IsFirstRun: " + isFirstRun);
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isFirstRun){
      m_Lift.setWristSetpoint(elevatorSetpoint);
      //System.out.println("Upper Arm is set to : " + Constants.Arm.RETRACT);
      isFirstRun = false;
      //System.out.println("IsFirstRun: " + isFirstRun);
    }


    if(Math.abs(Constants.wristTravel - m_Lift.getWristPosition()) < 0.2){
      m_Lift.setElevatorSetpoint(elevatorSetpoint);
      //System.out.println("Lower Arm is set to : " + lowerArmSetpoint);
    }

    if(Math.abs(elevatorSetpoint - m_Lift.getElevatorPosition()) < 0.3){
      m_Lift.setWristSetpoint(wristSetpoint);
      //System.out.println("Upper Arm is set to : " + upperArmSetpoint);
    }


    if((Math.abs(wristSetpoint - m_Lift.getWristPosition()) < 0.3) && (Math.abs(elevatorSetpoint - m_Lift.getElevatorPosition()) < 0.3)){
        //System.out.println("Complete Exiting Command");
        isFinished = true;
      }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
