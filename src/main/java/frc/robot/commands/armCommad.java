// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.beans.Encoder;
import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class armCommad extends CommandBase {
  /** Creates a new armCommad. */
  private final ArmSubsystem armSubsystem;
  private final Supplier<Double> speed;
  private final DigitalInput downSwitch = new DigitalInput(0);
  private final DigitalInput upSwitch = new DigitalInput(1);
  private final Encoder encoder = new Encoder();
  
  public armCommad(ArmSubsystem armSubsystem, Supplier<Double> speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.speed=speed;
    addRequirements(armSubsystem);
  }


// Called when the command is initially scheduled.
  @Override

  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realTimeSpeed = speed.get();
    if(downSwitch.get() == true && upSwitch.get() == true) {
      armSubsystem.setMotor(realTimeSpeed);
    } else if(downSwitch.get() == false && realTimeSpeed <= 0) {
      armSubsystem.setMotor(realTimeSpeed);
    } else if (upSwitch.get() == false && realTimeSpeed >= 0){
      armSubsystem.setMotor(realTimeSpeed);
    } else {
      armSubsystem.setMotor(0);
    }

    if(downSwitch.get() == false) {
      ArmSubsystem.resetEncoder();
    }

    // System.out.println("Encoder Position: " + ArmSubsystem.getPosition());



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
