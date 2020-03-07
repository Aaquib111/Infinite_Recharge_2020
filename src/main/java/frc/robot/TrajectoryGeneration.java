/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * Add your docs here.
 */
public class TrajectoryGeneration {
    private TrajectoryConfig config;
    private Drivetrain m_drive;

    //Constructor that generates a voltage constraint and a config
    public TrajectoryGeneration(Drivetrain m_drive){
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
        this.config = config;
        this.m_drive = m_drive;
    }

    /*  Autonomous trajectory:
        Move forward one unit
    */
    public Trajectory getAutonomousTrajectory(){
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                Arrays.asList(
                                new Pose2d(), 
                                new Pose2d(1.0, 0, new Rotation2d())),
                                config);
        return trajectory;
    }

    //Generic RamseteCommand generator
    public RamseteCommand generateRamseteCommand(Trajectory trajectory){
        return new RamseteCommand(
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
    }
}
