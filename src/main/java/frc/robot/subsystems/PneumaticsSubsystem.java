// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
  //private final DoubleSolenoid solenoidClaw = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0 , 1);
  private final DoubleSolenoid solenoidClaw = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  


  /** Creates a new pneumaticsSubsystem. */

  public PneumaticsSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
public void openandclose(boolean Open){
    if(Open){
      solenoidClaw.set(DoubleSolenoid.Value.kForward);
    } else {
      solenoidClaw.set(DoubleSolenoid.Value.kReverse);
    }



}

public void openandclose(boolean Open, double seconds){
  Timer.delay(seconds);
  if(Open){
    solenoidClaw.set(DoubleSolenoid.Value.kForward);
  } else {
    solenoidClaw.set(DoubleSolenoid.Value.kReverse);
  }



}

}
