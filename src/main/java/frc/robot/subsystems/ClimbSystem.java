/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSystem extends SubsystemBase {

  public CANSparkMax climber = new CANSparkMax(Constants.CLIMB_MOTOR, MotorType.kBrushless);
  public Spark winch = new Spark(Constants.WINCH);
  public DoubleSolenoid climbSolenoid = new DoubleSolenoid(Constants.CLIMB_EXTEND, Constants.CLIMB_RETRACT);
  
  public ClimbSystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runWinch(double speed){
    winch.set(speed);
  }

  public void runClimbMotor(double speed){
    climber.set(speed);
  }
}
