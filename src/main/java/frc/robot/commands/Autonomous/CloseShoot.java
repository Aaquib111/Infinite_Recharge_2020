/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.SimpleShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShootSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CloseShoot extends SequentialCommandGroup {
  /**
   * Creates a new CloseShoot.
   */
  public CloseShoot(Drivetrain d, ShootSystem s, double rpm) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new InstantCommand(d::setBrake),
      new InstantCommand(d::resetEncoders),
      new StartEndCommand(() -> d.arcadeDrive(0.5, 0), () -> d.arcadeDrive(0,0), d)
      .withInterrupt(() -> d.frontL.getEncoder().getPosition() > 1),
      new SimpleShoot(s, rpm)
    );

  }
}
