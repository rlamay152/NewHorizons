// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;



public class pneumaticsCommad extends CommandBase {

  private PneumaticsSubsystem PneumaticsSubsystem;
  public boolean open;


  public pneumaticsCommad(frc.robot.subsystems.PneumaticsSubsystem pneumaticsSubsystem, boolean open) {
    this.open = open;
    PneumaticsSubsystem = pneumaticsSubsystem;
    addRequirements(PneumaticsSubsystem);
}


  public void execute() {
    PneumaticsSubsystem.openandclose(open);
  }

  public boolean isFinished() {
    return false;
  }
  

}
