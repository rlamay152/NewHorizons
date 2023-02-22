// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.armCommad;
import frc.robot.commands.armSetPosition;
import frc.robot.commands.autonPneumaticsCommand;
import frc.robot.commands.changeSpeed;
import frc.robot.commands.pneumaticsCommad;
import frc.robot.commands.print;
import frc.robot.commands.selfRight;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();

  private final XboxController drivController = new XboxController(0);
  private final XboxController armController = new XboxController(1);
  private final JoystickButton zeroGyro = new JoystickButton(drivController, XboxController.Button.kStart.value);

  SendableChooser<Command> m_chooser = new SendableChooser<>();


  public RobotContainer() {
    // Set up the default command for the drivetrain.++
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis((drivController.getLeftY()) * Constants.driverSpeed) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis((drivController.getLeftX()) * Constants.driverSpeed) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis((drivController.getRightX()) * Constants.driverSpeed) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    m_chooser.setDefaultOption("Middle Auton", getAutonomousCommand());
    SmartDashboard.putData(m_chooser);

    // Configure the button bindings
    configureButtonBindings();
    armSubsystem.setDefaultCommand(new armCommad(armSubsystem, () -> armController.getRawAxis(XboxController.Axis.kRightY.value) * .65));
    //pneumaticsSubsystem.setDefaultCommand(new pneumaticsCommad(pneumaticsSubsystem, true));

    //System.out.println("Pose: "m_drivetrainSubsystem.getPose());


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope.
    zeroGyro.onTrue(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()));

    //closeGrabber.onTrue(new InstantCommand(() -> pneumaticsSubsystem.openandclose(false)));
    //openGrabber.onTrue(new InstantCommand(() -> pneumaticsSubsystem.openandclose(true)));
    new JoystickButton(drivController, XboxController.Button.kA.value).whileTrue(new selfRight(m_drivetrainSubsystem));
    new JoystickButton(armController, XboxController.Button.kX.value).onTrue(new pneumaticsCommad(pneumaticsSubsystem, true));
    new JoystickButton(armController, XboxController.Button.kB.value).onTrue(new pneumaticsCommad(pneumaticsSubsystem, false));
    new JoystickButton(armController, XboxController.Button.kY.value).whileTrue(new armSetPosition(armSubsystem, 2050));
    new JoystickButton(armController, XboxController.Button.kA.value).whileTrue(new armSetPosition(armSubsystem, 1706));
    new JoystickButton(drivController, XboxController.Button.kX.value).whileTrue(new changeSpeed());
    //new JoystickButton(drivController, XboxController.Button.kY.value).whileTrue(new alignVisonPID(m_drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // 1. Create trajectory settings

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND_TRAJECTORY, 
      DrivetrainSubsystem.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(DrivetrainSubsystem.m_kinematics);

    // 2. Generate trajectory
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, new Rotation2d(0)), 
    //   List.of(
    //     new Translation2d(0.69, 0.1)
    //     //new Translation2d(0.69, 0.1)

        
    //     // new Translation2d(0, 1)
    //     ), 
    //   new Pose2d(0.69, 0.1, Rotation2d.fromDegrees(0)), 
    //   trajectoryConfig);

    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(
        //new Translation2d(-1.0, -0.1),
        //new Translation2d(0.7, 0.1)
      ), 
      new Pose2d(0.5, 0.1, Rotation2d.fromDegrees(0)), 
      trajectoryConfig);

      Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
          new Translation2d(-2.5, -0.1)
        ), 
        new Pose2d(-1.0, 0.0, Rotation2d.fromDegrees(0)), 
        trajectoryConfig);

      // 3. Define PID controllers for tracking trajectory
      PIDController xController = new PIDController(Constants.kPXController, 0, 0);
      PIDController yController = new PIDController(Constants.kPYController, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.kPThetaConroller, 0, 0, Constants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      // 4. Construct command to follow trajectory

      SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
        trajectory1, 
        m_drivetrainSubsystem::getPose, 
        DrivetrainSubsystem.m_kinematics,
        xController,
        yController,
        thetaController,
        m_drivetrainSubsystem::drive, 
        m_drivetrainSubsystem);

        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
          trajectory2, 
          m_drivetrainSubsystem::getPose, 
          DrivetrainSubsystem.m_kinematics,
          xController,
          yController,
          thetaController,
          m_drivetrainSubsystem::drive, 
          m_drivetrainSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
          //new InstantCommand(() -> pneumaticsSubsystem.openandclose(false)),
          //new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()),
          new InstantCommand(() -> pneumaticsSubsystem.openandclose(false)),
          new WaitCommand(0.5),
          //new pneumaticsCommad(pneumaticsSubsystem, true),
         //new RunCommand (() -> armSubsystem.setMotor(autonPID.calculate(ArmSubsystem.getPosition(), 1706))),
          new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory1.getInitialPose())),
          //new print("Positioning Arm: "),
          new armSetPosition(armSubsystem, 1706),
          //new print("Moving: "),

          swerveControllerCommand1,

          //new InstantCommand(() -> pneumaticsSubsystem.openandclose(true, 2)),
          //new print("Opening Arm: "),
          new InstantCommand(() -> pneumaticsSubsystem.openandclose(true)),
          new WaitCommand(0.5),
          //new print("Moving Again: "),
          swerveControllerCommand2,
          new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
