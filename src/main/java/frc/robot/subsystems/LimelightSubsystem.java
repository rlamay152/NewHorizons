// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class LimelightSubsystem extends SubsystemBase {
  NetworkTable table;
  double tx;
  double ty;
  double ta;
  /** Creates a new Vision. */
  public LimelightSubsystem() {
    tx = table.getEntry("tx").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);

  }
  public double getYdeviation(){
    return table.getEntry("ty").getDouble(0.0);
  }
  public double getXdeviation(){
    return table.getEntry("tx").getDouble(0.0);
  }
     public void enableVisionProcessing(){
    table.getEntry("camMode").setNumber(0);
    System.out.println("Vision processing enabled");
  } 
  public void enableDriverCamera(){
    table.getEntry("camMode").setNumber(1);
     table.getEntry("camMode").setNumber(0);
  
  }

  public BooleanSupplier visionProcessing(){
    BooleanSupplier camMode = () -> table.getEntry("camMode").getDouble(1) == 0;
    return camMode;
    
  }
  public double getCamMode(){
    return table.getEntry("camMode").getDouble(0);
  }
  public void midNodeTarget(){
    table.getEntry("pipeline").setNumber(0);
  }

  public void highNodeTarget(){
    table.getEntry("pipeline").setNumber(1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
