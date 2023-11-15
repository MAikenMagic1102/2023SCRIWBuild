// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Gamepiece;
import frc.robot.Gamepiece.GamepieceType;


public class Lift extends SubsystemBase {
  /** Creates a new Lift. */
  TalonFX elevator;

  TalonFX wrist;
  DigitalInput upperLimit;
  DigitalInput lowerLimit;
  boolean systemZeroed = false;

  double elevatorSetpoint = 2.0;
  double wristSetpoint = 0.0;

  public Lift() {
    elevator = new TalonFX(Constants.elevator, "can2");
    wrist = new TalonFX(Constants.wrist);
    upperLimit = new DigitalInput(Constants.upperLimit);
    lowerLimit = new DigitalInput(Constants.lowerLimit);

    TalonFXConfiguration elevatorConfigs = new TalonFXConfiguration();
    elevatorConfigs.Slot0.kP = Constants.elevator_kP; // An error of 0.5 rotations results in 12V output
    elevatorConfigs.Slot0.kD = Constants.elevator_kD; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    elevatorConfigs.Voltage.PeakForwardVoltage = 8;
    elevatorConfigs.Voltage.PeakReverseVoltage = -8;
    
    // Peak output of 130 amps
    elevatorConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 130;
    elevatorConfigs.TorqueCurrent.PeakReverseTorqueCurrent = 130;

    // elevatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // elevatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.2;
    // elevatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // elevatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 140.5;

    TalonFXConfiguration wristConfigs = new TalonFXConfiguration();
    wristConfigs.Slot0.kP = Constants.wrist_kP; // An error of 0.5 rotations results in 12V output
    wristConfigs.Slot0.kD = Constants.wrist_kD; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    wristConfigs.Voltage.PeakForwardVoltage = 8;
    wristConfigs.Voltage.PeakReverseVoltage = -8;
    
    // Peak output of 130 amps
    wristConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 130;
    wristConfigs.TorqueCurrent.PeakReverseTorqueCurrent = 130;

    // wristConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // wristConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 23.0;
    // wristConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // wristConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.5;

    elevatorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode wristStatus = StatusCode.StatusCodeNotInitialized;
    StatusCode elevatorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      elevatorStatus = elevator.getConfigurator().apply(elevatorConfigs);
      wristStatus = wrist.getConfigurator().apply(wristConfigs);
      if (elevatorStatus.isOK() && wristStatus.isOK()) break;
    }
    if(!elevatorStatus.isOK() && !wristStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + elevatorStatus.toString() + " " + wristStatus.toString());
    }

  }

  
  public void setElevatorOpenLoop(double elevatorDemand){
    elevator.set(elevatorDemand);
  }

  public void setWristOpenLoop(double wristDemand){
    wrist.set(wristDemand);
  }

  public void setOpenLoop(double elevatorDemand, double wristDemand){
    setElevatorOpenLoop(elevatorDemand);
    setWristOpenLoop(wristDemand);
  }

  public void setClosedLoopElevatorPosition(double elevatorPosition){
    PositionVoltage request = new PositionVoltage(elevatorPosition);
    elevator.setControl(request);
  }

  public void setClosedLoopWristPosition(double wristPosition){
    PositionVoltage request = new PositionVoltage(wristPosition);
    wrist.setControl(request);
  }

  public void setClosedLoopPosition(double elevatorPosition, double wristPosition){
    setClosedLoopElevatorPosition(elevatorPosition);
    setClosedLoopWristPosition(wristPosition);
  }



  public void initialize(){
    if(!systemZeroed && elevator.getStatorCurrent().getValueAsDouble() < 20.0){
      setElevatorOpenLoop(-0.1);
    }else{
      setElevatorOpenLoop(0.0);
      elevator.setPosition(0);
      systemZeroed = true;
    }
  }

  public void setElevatorSetpoint(double setpoint){
    elevatorSetpoint = setpoint;
  }

  public void setWristSetpoint(double setpoint){
    wristSetpoint = setpoint;
  }

  public void setLiftSetpoint(double elevatorSetpoint, double wristSetpoint){
    this.elevatorSetpoint = elevatorSetpoint;
    this.wristSetpoint = wristSetpoint;
  }

  public double getElevatorSetpoint(){
    return elevatorSetpoint;
  }

  public double getWristSetpoint(){
    return wristSetpoint;
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

  public boolean getSystemZeroed(){
    return systemZeroed;
  }

  /////////////////////////////////////////
  // public CommandBase liftControl(double wristDemand, double elevatorDemand){
  //   if(!systemZeroed){
  //     return this.runOnce(
  //       () -> initialize()
  //     );
  //   }else{
  //     return this.runOnce(
  //       () -> setOpenLoop(wristDemand, elevatorDemand)
  //     );
  //   }
  // }

  public CommandBase floorPickup(){
    if(Gamepiece.currentGamepiece == GamepieceType.Cone){
      return this.runOnce(
        () -> setLiftSetpoint(Constants.elevatorGroundCone, Constants.wristGroundCone)
      );
    }else{
      return this.runOnce(
        () -> setLiftSetpoint(Constants.elevatorGroundCube, Constants.wristGroundCube)
      );
    }
  }

  public CommandBase scoreLow(){
    if(Gamepiece.currentGamepiece == GamepieceType.Cone){
      return this.runOnce(
        () -> setLiftSetpoint(Constants.elevatorScoreConeBottom, Constants.wristScoreConeBottom)
      );
    }else{
      return this.runOnce(
        () -> setLiftSetpoint(Constants.elevatorScoreCubeBottom, Constants.wristScoreCubeBottom)
      );
    }
  }

  public CommandBase scoreMiddle(){
    if(Gamepiece.currentGamepiece == GamepieceType.Cone){
      return this.runOnce(
        () -> setLiftSetpoint(Constants.elevatorScoreConeMiddle, Constants.wristScoreConeMiddle)
      );
    }else{
      return this.runOnce(
        () -> setLiftSetpoint(Constants.elevatorScoreCubeMiddle, Constants.wristScoreCubeMiddle)
      );
    }
  }

  public CommandBase scoreHigh(){
    return this.runOnce(
      () -> setLiftSetpoint(Constants.elevatorScoreCubeTop, Constants.wristScoreCubeTop)
    );
  }
  
  public CommandBase liftStow(){
    return this.runOnce(
      () -> setLiftSetpoint(Constants.elevatorStow, Constants.wristStow)
    );
  }
   

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
    SmartDashboard.putNumber("Elevator Error", getElevatorError());
    SmartDashboard.putNumber("Elevator Setpoint", elevatorSetpoint);
    SmartDashboard.putNumber("Wrist Position", getWristPosition());
    SmartDashboard.putNumber("Wrist Error", getWristError());
    SmartDashboard.putNumber("Wrist Setpoint", wristSetpoint);
    SmartDashboard.putNumber("Elevator Stator Current", elevator.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator Zeroed", systemZeroed);
  }
}
