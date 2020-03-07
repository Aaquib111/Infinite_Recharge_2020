/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Shoot;
import frc.robot.commands.ToggleSolenoid;
import frc.robot.commands.TrackTarget;
import frc.robot.commands.Autonomous.CloseShoot;
import frc.robot.commands.Autonomous.SimpleAuto;
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

  public TrajectoryGeneration m_trajectoryGenerator = new TrajectoryGeneration(m_drive);

  private Joystick left = new Joystick(Constants.LEFT_STICK);
  private Joystick right = new Joystick(Constants.RIGHT_STICK);
  private Joystick gamepad = new Joystick(Constants.GAMEPAD);

  private Command m_simpleAuto = new SimpleAuto(m_drive, m_shoot, m_limelight);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new RunCommand(
      () -> m_drive.tankDrive(gamepad.getRawAxis(1), gamepad.getRawAxis(5)), m_drive));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton runIntake = new JoystickButton(right, Constants.INTAKE_BTN);
      runIntake.whileHeld(new StartEndCommand(() -> m_shoot.runIntake(-0.5), () -> m_shoot.runIntake(0), m_shoot));
    final JoystickButton toggleIntake = new JoystickButton(gamepad, Constants.INTAKE_SOL_BTN);
      toggleIntake.whenPressed(new ToggleSolenoid(m_shoot.intakeSolenoid));
    final JoystickButton reverse = new JoystickButton(gamepad, Constants.INTAKE_REVERSE_BTN);
      reverse.whileHeld(new StartEndCommand(() -> m_shoot.runIntake(.5), () -> m_shoot.runIntake(0), m_shoot));
    final JoystickButton feed = new JoystickButton(gamepad, Constants.FEEDER_BTN);
      feed.whileHeld(new StartEndCommand(() -> m_shoot.runFeed(0.5), () -> m_shoot.runFeed(0), m_shoot));
    final JoystickButton feedReverse = new JoystickButton(gamepad, Constants.FEEDER_BTN);
      feedReverse.whileHeld(new StartEndCommand(() -> m_shoot.runFeed(-0.5), () -> m_shoot.runFeed(0), m_shoot));
    final JoystickButton aim = new JoystickButton(left, Constants.AIM_BTN);
      aim.whileHeld(new TrackTarget(m_drive));
    final JoystickButton shoot = new JoystickButton(gamepad, Constants.FAR_SHOOT_BTN);
      shoot.whileHeld(new Shoot(m_shoot, m_limelight));
    final JoystickButton toggleShoot = new JoystickButton(gamepad, Constants.SWITCH_SHOOT_BTN);
      toggleShoot.whenPressed(new ToggleSolenoid(m_shoot.shootSolenoid));
    final JoystickButton closeShoot = new JoystickButton(gamepad, Constants.CLOSE_SHOOT_BTN);
      closeShoot.whileHeld(new CloseShoot(m_drive, m_shoot, 3700));

    final JoystickButton runArm = new JoystickButton(gamepad, Constants.ARM_BTM);
      runArm.whileHeld(new StartEndCommand(() -> m_climb.runClimbMotor(0.5), () -> m_climb.runClimbMotor(0), m_climb));
    final JoystickButton runWinch = new JoystickButton(gamepad, Constants.WINCH_BTN);
      runWinch.whileHeld(new StartEndCommand(() -> m_climb.runWinch(0.5), () -> m_climb.runWinch(0), m_climb));
    final JoystickButton toggleClimb = new JoystickButton(gamepad, Constants.ARMSOL_BTN);
      toggleClimb.whenPressed(new ToggleSolenoid(m_shoot.shootSolenoid));

    final JoystickButton runPanel = new JoystickButton(gamepad, Constants.CP_BTN);
      runPanel.whileHeld(new StartEndCommand(() -> m_control.run(0.5), () -> m_control.run(0), m_control));
    final JoystickButton togglePanel = new JoystickButton(gamepad, Constants.CPSOL_BTN);
      togglePanel.whenPressed(new ToggleSolenoid(m_control.controlSolenoid));

    final JoystickButton toggleDriveMode = new JoystickButton(gamepad, Constants.DRIVE_MODE_BTN);
      toggleDriveMode.whenPressed(new InstantCommand(m_drive::switchDrive, m_drive));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {     
    Trajectory trajectory = m_trajectoryGenerator.getAutonomousTrajectory();
    RamseteCommand command = m_trajectoryGenerator.generateRamseteCommand(trajectory);

    return command.andThen(() -> m_drive.setOutput(0, 0));
    //return m_simpleAuto;
  }
}
