// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.tools.DiagnosticCollector;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */
  TalonFX elevator;
  TalonFX wrist;
  DigitalInput upperlimit;
  DigitalInput lowerlimit; 

  public Lift() {
    elevator = new TalonFX(Constants.elevator);
    wrist = new TalonFX(Constants.wrist);
    upperlimit = new DigitalInput (Constants.upperlimit);
    lowerlimit = new DigitalInput (Constants.lowerlimit);

    }
    public void setOpenLoop (double wristDemand, double elevatorDemand){
        elevator.set(elevatorDemand);
    }
    public void setClosedLoopPosition(double elevatorPosition, double wristPosition){
      double Position;
      PositionVoltage request = new PositionVoltage(2.0);
      elevator.setControl(request);
    }


    public double getelevatorError(){
      return elevator.getClosedLoopError().getValueAsDouble();
    }

    public double getwristError(){
      return wrist.getClosedLoopError().getValueAsDouble();
    }

    public double getElevatorPosition(){
      return elevator.getRotorPosition().getValueAsDouble(); 
    }

    public double getWristPosition(){
      return elevator.getRotorPosition().getValueAsDouble();
    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    

  }
}
