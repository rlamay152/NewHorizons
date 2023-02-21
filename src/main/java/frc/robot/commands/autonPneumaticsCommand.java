// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;



public class autonPneumaticsCommand extends CommandBase {

  private PneumaticsSubsystem PneumaticsSubsystem;
  public boolean open;
  private final Timer m_timer = new Timer();


  public autonPneumaticsCommand(frc.robot.subsystems.PneumaticsSubsystem pneumaticsSubsystem, boolean open) {
    this.open = open;
    PneumaticsSubsystem = pneumaticsSubsystem;
    addRequirements(PneumaticsSubsystem);
}
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  public void execute() {
    PneumaticsSubsystem.openandclose(open);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  public boolean isFinished() {
    return m_timer.hasElapsed(1.5);
  }
  

}
