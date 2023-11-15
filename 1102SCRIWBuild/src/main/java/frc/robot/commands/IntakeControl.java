// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Gamepiece;
import frc.robot.Gamepiece.GamepieceType;
import frc.robot.subsystems.Intake;

public class IntakeControl extends CommandBase {
  /** Creates a new IntakeControl. */
  Intake intake;
  BooleanSupplier inSup;
  BooleanSupplier outSup;

  boolean in, out;

  public IntakeControl(Intake intake, BooleanSupplier inSup, BooleanSupplier outSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.inSup = inSup;
    this.outSup = outSup;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    in = inSup.getAsBoolean();
    out = outSup.getAsBoolean();

    if(in){
      if(Gamepiece.getGamepiece() == GamepieceType.Cube){
        intake.intakeInCube();
      }else{
        //Gamepiece.currentGamepiece == GamepieceType.Cone
        intake.intakeInCone();
      }
    }else if(out){
      if(Gamepiece.getGamepiece() == GamepieceType.Cube){
        intake.intakeReleaseCube();
      }else{
        //Gamepiece.currentGamepiece == GamepieceType.Cone
        intake.intakeReleaseCone();
      }
    }else{
      if(Gamepiece.getGamepiece() == GamepieceType.Cube){
        intake.intakeHoldCube();
      }else{
        //Gamepiece.currentGamepiece == GamepieceType.Cone
        intake.intakeHoldCone();
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
