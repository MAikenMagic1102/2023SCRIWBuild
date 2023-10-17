// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  TalonFX vertRoller;
  CANSparkMax hRoller;

  PneumaticHub hub;
  Solenoid single;

  DigitalInput beamSensor;

  boolean hold = false;

  public Intake() {
    vertRoller = new TalonFX(Constants.vertRoller);
    hRoller = new CANSparkMax(Constants.horiRoller, MotorType.kBrushless);

    hRoller.setSmartCurrentLimit(Constants.hRollerCurrentLimit);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.vertRollerCurrentLimit;

    hub = new PneumaticHub();
    single = new Solenoid(PneumaticsModuleType.REVPH, Constants.intakeSolenoid);

    beamSensor = new DigitalInput(Constants.beamSensor);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = vertRoller.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void setvertOpenLoop(double vertRollerDemand){
    vertRoller.set(vertRollerDemand);
  }

  public void setHRollerOpenLoop(double hRollerDemand){
    hRoller.set(hRollerDemand);
  }

  public double getVelocity(){
    return vertRoller.getRotorVelocity().getValueAsDouble();
  }

  public boolean getBeamSensor(){
    return beamSensor.get();
  }

  public void setOpen(){
    single.set(true);
  }

  public void setClosed(){
    single.set(false);
  }

  public void intakeIn(){
    setOpen();
    setvertOpenLoop(-0.5);
  }

  public void intakeInCube(){
	setOpen();
    if (getBeamSensor()){
		setvertOpenLoop(-0.5);
		setHRollerOpenLoop(-0.5);
    }else {
		setvertOpenLoop(0);
		setHRollerOpenLoop(0);
	}
  }

   public void intakeInHold(){
     setOpen();
     if (!getBeamSensor()){
       setvertOpenLoop(-0.05);
       setHRollerOpenLoop(-0.05);
     }else {
       setvertOpenLoop(0);
       setHRollerOpenLoop(0);
     }
   }

	public void intakeHold(){
		setClosed();
		setvertOpenLoop(0.01);
	}
	
	public void intakeRelease(){
		setOpen();
	}
	
	public void intakeReleaseCube(){
		setvertOpenLoop(0.5);
		setOpen();
	}
	
	public void intakeLaunchCube(){
		setvertOpenLoop(1.0);
		setOpen();
	}
	
	public CommandBase intakeHoldCommand(){
		return this.runOnce(
		() -> intakeHold()
		);
	}
	
	public CommandBase intakeReleaseCommand(){
		return this.runOnce(
		() -> intakeRelease()
		); 
	}
	
	public CommandBase intakeReleaseCubeCommand(){
		return this.runOnce(
		() -> intakeReleaseCube()
		);
	} 
	
	public CommandBase intakeLaunchCubeCommand(){
		return this.runOnce(
		() -> intakeLaunchCube()
		);
	}
	
	public CommandBase intakeInCommand(){
	return this.runOnce(
		() -> intakeIn()
	);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("VertRoller Velocity", getVelocity());
    SmartDashboard.putBoolean("Beam Sensor", getBeamSensor());
  }
}

