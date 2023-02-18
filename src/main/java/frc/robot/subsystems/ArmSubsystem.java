// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.beans.Encoder;

import javax.swing.text.TabSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {


    private final static TalonSRX armTalon = new TalonSRX(8);
    

    GenericEntry botNode;
    GenericEntry ArmAngle;

   //  ShuffleboardTab shuffleArm;
    // private final RelativeEncoder m_rotationEncoder;


/** Creates a new ExampleSubsystem. */
public ArmSubsystem() {

 }
   //shuffleArm = Shuffleboard.getTab(Shuffleboard.kBaseTableName);

//  public double getRotationEncoder() {
//    return armEncoder.get();
//  }

//   public double getRotationAngle() {
//     return getRotationEncoder();
//   }


/**
 * Example command factory method.
 *
 * @return a command
 */
public CommandBase exampleMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return runOnce(
      () -> {
        /* one-time action goes here */
      });
}

/**
 * An example method querying a boolean state of the subsystem (for example, a digital sensor).
 *
 * @return value of some boolean subsystem state, such as a digital sensor.
 */
public boolean exampleCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}


/* (non-Javadoc)
 * @see edu.wpi.first.wpilibj2.command.Subsystem#periodic()
 */
@Override
public void periodic() { 
//ArmAngle.setDouble(getArmRotationDegrees());
  

  // This method will be called once per scheduler run




}

public static void configFactory() {
  armTalon.configFactoryDefault();
}

// public static void setBrake() {
//   armTalon.setNeutralMode(NeutralMode.Brake);
// }

// public static void setCoast() {
//   armTalon.setNeutralMode(NeutralMode.Coast);
// }

public static double getPosition() {
  return armTalon.getSelectedSensorPosition();
}

public static void resetEncoder() {
  armTalon.setSelectedSensorPosition(0);
}

@Override
public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
}



public void setMotor(double speed) {
  armTalon.set(ControlMode.PercentOutput, speed);
}
}
