// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */
  TalonFX elevator;
    TalonFX wrist;
     DigitalInput Upperlimit;
     DigitalInput Lowerlimit; 

  public Lift() {
   elevator = new TalonFX(Constants.elevator);
   elevator = new TalonFX(Constants.wrist);
   Upperlimit = new DigitalInput(Constants.Upperlimit);
   Lowerlimit = new DigitalInput(Constants.Lowerlimit); 
  }

  public void setOpenLoop(double wristDemand,double elevatorDemand){
    elevator.set(elevatorDemand);
    wrist.set(wristDemand);
  }

  public void setClosedLoop(double elevatorPosition,double wristPosition){
    PositionVoltage request = new PositionVoltage(2.0);
    elevator.setControl(request);
  }
  // double getelevatorError(4.0);
  // double getwristError(4.0);
  
  public double getelevatorError(){
    return elevator.getClosedLoopError().getValueAsDouble();
  }

  public double getwristError(){
    return wrist.getClosedLoopError().getValueAsDouble();
  }

  public double getElevatorPosition(){
    return elevator.getRotorPosition().getValueAsDouble();
    
  }

  public double getwristposition(){
    return wrist.getRotorPosition().getValueAsDouble();

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
