/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  /**
   * Creates a new Limelight.
   */
  public Limelight() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Targets", getValidTargets());
    SmartDashboard.putNumber("Distance", getDistance());
  }

  NetworkTableEntry ledMode = Constants.ledMode;
  NetworkTableEntry camMode = Constants.camMode;
  NetworkTableEntry pipeline = Constants.pipeline;
  public NetworkTableEntry stream = Constants.stream;

  double validTarget = Constants.tv.getDouble(0.0);


  /**
   * Sets limelight to vision processing mode and sets vision pipeline to 0
   */
  public void visionMode() {
    camMode.setNumber(0); // sets camera to vision processing mode
    pipeline.setNumber(0);
  }

  public void driverMode() {
    camMode.setNumber(1); // sets camera to driving mode
    pipeline.setNumber(0);
  }

  /**
   * Forces on light
   */
  public void lightOn() {
    ledMode.setNumber(3);
  }

  /**
   * Forces off light
   */
  public void lightOff() {
    ledMode.setNumber(1);
  }

  /**
   * Changes light settings according to how vision pipeline is set
   */
  public void lightAuto() {
    ledMode.setNumber(0);
  }
    
  /**
   * Checks if there is no valid targets, which is then sent to isFinsished() 
   */
  public boolean noValidTarget() {
    if (validTarget == 0) {
      SmartDashboard.putBoolean("Target", false);
      return true; 

    } else {
      SmartDashboard.putBoolean("Target", true);
      return false;
      
    }
  }

  public boolean isValidTarget(){
    if(Constants.tv.getDouble(0.0) == 1){
      return true;
    }
    else{
      return false;
    }
  }

  private double getDistance(){
    double radians = Math.toRadians(Constants.ty.getDouble(0.0));
    return (Constants.powerPortHeight - Constants.limelightHeight) / Math.tan(Constants.limelightAngle + radians);
  }

  //EQUATION GOES HERE
  //Right now: f(x) = .5x + 2000 <- please change 

  public double getRpm(){
    return 0.5 * getDistance() + 2000;
  }
  private boolean getValidTargets(){
    return Constants.tv.getDouble(0.) == 0 ? false : true;
  }
}
