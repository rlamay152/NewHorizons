// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class armSetPosition extends CommandBase {
  /** Creates a new armSetPosition. */
  private final ArmSubsystem armSubsystem;
  public static final PIDController pid = new PIDController(Constants.kPArm, Constants.kIArm, 0);
  //public static final PIDController autonPID = new PIDController(Constants.kPArmAuton, Constants.kIArmAuto, 0);
  public final double position;
  public armSetPosition(ArmSubsystem armSubsystem, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.position = position;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setMotor(-pid.calculate(ArmSubsystem.getPosition(), position) * 0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ArmSubsystem.getPosition() >= position;
  }
}
