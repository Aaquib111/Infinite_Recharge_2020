/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootSystem extends SubsystemBase {

  public DoubleSolenoid shootSolenoid = new DoubleSolenoid(Constants.SHOOTER_FORWARD, Constants.SHOOTER_REVERSE);
  
  public CANSparkMax shoot = new CANSparkMax(Constants.SHOOT_MOTOR, MotorType.kBrushless);
  public CANSparkMax feed = new CANSparkMax(Constants.FEED_MOTOR, MotorType.kBrushless);
  public CANSparkMax intake = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);

  public ShootSystem() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Shooting: ", getShooterPosition());
    SmartDashboard.putNumber("Shooter RPM", shoot.getEncoder().getVelocity());
  }

  //BANG-BANG IMPLEMENTATION
  public void shoot(double rpm){
    if(shoot.getEncoder().getVelocity() < rpm){
      shoot.set(1);
    }else{
      feed.set(.7);
    }
  }

  public void runIntake(double speed){
    intake.set(speed);
  }

  public void runFeed(double speed){
    feed.set(speed);
  }

  public String getShooterPosition(){
    if(shootSolenoid.get() == Value.kForward){
      return "Close";
    }
    else{
      return "Far";
    }
  }
  
  /*TAKE BACK HALF CONTROL 
    e = S-P;                            // calculate the error;
    Y += G*e;                           // integrate the output;
    if (Y>1) Y=1; else if (Y<0) Y=0;    // clamp the output to 0..+1;
    if (signbit(e)!=signbit(d)){        // if zero crossing,
      Y = b = 0.5*(Y+b);                // then Take Back Half
      d = e;}                           // and save the previous error;
  */
}
