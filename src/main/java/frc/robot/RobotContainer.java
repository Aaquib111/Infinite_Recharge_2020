/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.ControlPanelSystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShootSystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public Drivetrain m_drive = new Drivetrain();
  public ShootSystem m_shoot = new ShootSystem();
  public Limelight m_limelight = new Limelight();
  public ClimbSystem m_climb = new ClimbSystem();
  public ControlPanelSystem m_control = new ControlPanelSystem();

  private Joystick left = new Joystick(Constants.LEFT_STICK);
  private Joystick right = new Joystick(Constants.RIGHT_STICK);
  private Joystick gamepad = new Joystick(Constants.GAMEPAD);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new RunCommand(
      () -> m_drive.tankDrive(left.getRawAxis(1), right.getRawAxis(1))));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.kMaxSpeedMetersPerSecond,
                                   Constants.kMaxAccelerationMetersPerSecondSquared,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        m_drive.getKinematics(),
        10);

    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(11), Units.feetToMeters(2));
    config.setKinematics(m_drive.getKinematics());
    config.addConstraint(autoVoltageConstraint);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                              Arrays.asList(
                              new Pose2d(), 
                              new Pose2d(1.0, 0, new Rotation2d())
                              ),
                              config);
                              
    RamseteCommand command = new RamseteCommand(
    trajectory, 
    m_drive::getPose, 
    new RamseteController(2.,.7),
    m_drive.getFeedForward(),
    m_drive.getKinematics(),
    m_drive::getSpeeds,
    m_drive.getLeftPIDController(), 
    m_drive.getRightPIDController(),
    m_drive::setOutput, 
    m_drive);

    //System.out.println(command);
    return command.andThen(() -> m_drive.setOutput(0, 0));
  }
}
