/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShootSystem;

public class Shoot extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private ShootSystem s;
  private Limelight l;
  private double rpm;

  public Shoot(ShootSystem s, Limelight l) {
    this.s = s;
    this.l = l;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rpm = l.getRpm();
    if(s.shoot.getEncoder().getVelocity() < rpm){
      s.shoot.set(1);
    }else{
      s.shoot.set(0);
      s.feed.set(.7);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s.shoot.set(0);
    s.feed.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
