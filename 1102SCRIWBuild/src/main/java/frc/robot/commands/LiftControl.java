// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;

public class LiftControl extends CommandBase {
  /** Creates a new LiftControl. */
  Lift myLift;
  double elevatorDemand;
  double wristDemand;
  boolean resetHoldElevator = false;
  boolean resetHoldWrist = false;

  public LiftControl(Lift p_lift, DoubleSupplier elevatorSup, DoubleSupplier wristSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    myLift = p_lift;
    elevatorDemand = elevatorSup.getAsDouble();
    wristDemand = wristSup.getAsDouble();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!myLift.getSystemZeroed()){
      myLift.initialize();
    }else{
      if(MathUtil.applyDeadband(elevatorDemand, Constants.deadband) > 0){
        myLift.setElevatorOpenLoop(elevatorDemand);
        resetHoldElevator = true;
      }else{
        if(resetHoldElevator){
          myLift.setElevatorSetpoint(myLift.getElevatorPosition());
          resetHoldElevator = false;
        }

        if(!resetHoldElevator){
          myLift.setClosedLoopElevatorPosition(myLift.getElevatorSetpoint());
        }else{
          myLift.setElevatorOpenLoop(0);
        }
      }

      if(MathUtil.applyDeadband(wristDemand, Constants.deadband) > 0){
        myLift.setWristOpenLoop(wristDemand);
        resetHoldWrist = true;
      }else{
        if(resetHoldWrist){
          myLift.setWristSetpoint(myLift.getWristPosition());
          resetHoldWrist = false;
        }

        if(!resetHoldWrist){
          myLift.setClosedLoopWristPosition(myLift.getWristSetpoint());
        }else{
          myLift.setWristOpenLoop(0);
        }
      }
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
