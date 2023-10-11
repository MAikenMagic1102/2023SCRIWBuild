// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */
  TalonFX elevator;
  TalonFX wrist;
  DigitalInput upperLimit;
  DigitalInput lowerLimit;
  public Lift() {
    elevator = new TalonFX(Constants.elevator);
    wrist = new TalonFX(Constants.wrist);
    upperLimit = new DigitalInput(Constants.upperLimit);
    lowerLimit = new DigitalInput(Constants.lowerLimit);
  }
  public void setOpenLoop(double wristDemand, double elevatorDemand){
    elevator.set(elevatorDemand);
    wrist.set(wristDemand);
  }
  public void setClosedLoopPosition(double evevatorPosition, double wristPosition){
    PositionVoltage request = new PositionVoltage(2.0);
    elevator.setControl(request);
    wrist.setControl(request);
  }

  public double getElevatorPosition(){
    return elevator.getPosition().getValueAsDouble();
  }
  public double getWristPosition(){
    return wrist.getPosition().getValueAsDouble();
  }

  public double getElevatorError(){
    return elevator.getClosedLoopError().getValueAsDouble();
  }
  public double getWristError(){
    return wrist.getClosedLoopError().getValueAsDouble();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
    SmartDashboard.putNumber("Elevator Error", getElevatorError());
    SmartDashboard.putNumber("Wrist Position", getWristPosition());
    SmartDashboard.putNumber("Wrist Error", getWristError());
  }
}
