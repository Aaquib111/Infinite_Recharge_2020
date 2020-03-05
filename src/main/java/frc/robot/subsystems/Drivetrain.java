/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    
  public CANSparkMax frontL = new CANSparkMax(Constants.FRONTL, MotorType.kBrushless);
  public CANSparkMax backL = new CANSparkMax(Constants.BACKL, MotorType.kBrushless);  
  public CANSparkMax frontR = new CANSparkMax(Constants.FRONTR, MotorType.kBrushless);  
  public CANSparkMax backR = new CANSparkMax(Constants.BACKR, MotorType.kBrushless);  
  
  private SpeedControllerGroup left = new SpeedControllerGroup(frontL, backL);
  private SpeedControllerGroup right = new SpeedControllerGroup(frontR, backR);

  private DifferentialDrive drive = new DifferentialDrive(left, right);

  private AHRS navx = new AHRS(I2C.Port.kMXP);

  public final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);

  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getHeading());
  
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter
  );

  private PIDController leftPidController = new PIDController(9.95, 0, 0);
  private PIDController rightPidController = new PIDController(9.95, 0, 0);

  private Pose2d pose;

  public Drivetrain() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Heading", navx.getAngle());
    SmartDashboard.putString("Idle Mode: ", getIdleMode());
    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("Right Distance", getRightDistance());

    pose = m_odometry.update(getHeading(), getLeftDistance(), getRightDistance());
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double speed, double rot){
    drive.arcadeDrive(speed, rot);
  }

  public String getIdleMode(){
    if(frontL.getIdleMode() == IdleMode.kCoast){
      return "Coast";
    }else{
      return "Brake";
    }
  }

  //FALSE = NAVX NOT INVERTED
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(navx.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      frontL.getEncoder().getVelocity() / 10.74 * 2 * Math.PI * Units.inchesToMeters(4) / 60,
      frontR.getEncoder().getVelocity() / 10.74 * 2 * Math.PI * Units.inchesToMeters(4) / 60
    );
  }

  //Honestly idk if its supposed to be .getPosition() or .getVelocity() but well test it out
  public double getLeftDistance(){
    return frontL.getEncoder().getPosition() / 10.74 * 2 * Math.PI * Units.inchesToMeters(4);
  }

  //Honestly idk if its supposed to be .getPosition() or .getVelocity() but well test it out
  public double getRightDistance(){
    return frontL.getEncoder().getPosition() / 10.74 * 2 * Math.PI * Units.inchesToMeters(4);
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedforward;
  }

  public PIDController getLeftPIDController(){
    return leftPidController;
  }

  public PIDController getRightPIDController(){
    return rightPidController;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kDriveKinematics;
  }

  public Pose2d getPose(){
    return pose;
  }

  public void setOutput(double leftVolts, double rightVolts){
    frontL.setVoltage(leftVolts / 2);
    frontR.setVoltage(rightVolts / 2);
    drive.feed();
  }

  public void switchDrive(){
    if(frontL.getIdleMode() == IdleMode.kBrake){
      frontL.setIdleMode(IdleMode.kCoast);
      frontR.setIdleMode(IdleMode.kCoast);
      backL.setIdleMode(IdleMode.kCoast);
      backR.setIdleMode(IdleMode.kCoast);
    }else{
      frontL.setIdleMode(IdleMode.kBrake);
      frontR.setIdleMode(IdleMode.kBrake);
      backL.setIdleMode(IdleMode.kBrake);
      backR.setIdleMode(IdleMode.kBrake);
    }
  }
}
