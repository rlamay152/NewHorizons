// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class alignVisonPID extends CommandBase {
  private final DrivetrainSubsystem m_DriveTrainSubsystem;

  private final PIDController pid = new PIDController(Constants.kPLimelite, 0, 0);
  private final LimelightSubsystem limelite = new LimelightSubsystem();


  public alignVisonPID(DrivetrainSubsystem m_driveTrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_DriveTrainSubsystem = m_driveTrainSubsystem;
    
    addRequirements(m_driveTrainSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveTrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      pid.calculate(limelite.getXdeviation(), 0),
      pid.calculate(limelite.getYdeviation(), 0),
      0,
      m_DriveTrainSubsystem.getGyroscopeRotation()
       ));
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      0, 0, 0, m_DriveTrainSubsystem.getGyroscopeRotation()
    ));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}