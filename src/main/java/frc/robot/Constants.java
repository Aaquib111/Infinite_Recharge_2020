/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    //USB
    public static final int LEFT_STICK = 2;
    public static final int RIGHT_STICK = 1;
    public static final int GAMEPAD = 0;

    //CANSPARKMAX
    public static final int FRONTL = 3;
    public static final int FRONTR = 1;
    public static final int BACKL = 4;
    public static final int BACKR = 2;

    public static final int SHOOT_MOTOR = 8;
    public static final int FEED_MOTOR = 7;
    public static final int INTAKE_MOTOR = 5;
    
    public static final int CONTROL_PANEL_MOTOR = 6;

    public static final int WINCH = 0;
    public static final int CLIMB_MOTOR = 9;

    //JOYSTICK BUTTONS
    public static final int INTAKE_BTN = 1;
    public static final int FAR_SHOOT_BTN = 14;
    public static final int INTAKE_SOL_BTN = 3;
    public static final int CLOSE_SHOOT_BTN = 4;
    public static final int SWITCH_SHOOT = 5;
    public static final int CPSOL_BTN = 6;
    public static final int FEEDER_BTN = 13;
    public static final int ARMSOL_BTN = 10;
    public static final int ARMMOTOR_BTN = 9;
    public static final int WINCH_BTN = 10;
    public static final int CP_BTN = 11;
    public static final int DRIVE_MODE_BTN = 6;
    public static final int FEED_REVERSE_BTN = 12;
    public static final int AIM_BUTTON = 2;

    //DOUBLE SOLENOIDS
    public static final int INTAKE_FORWARD = 4;
    public static final int INTAKE_REVERSE = 5;

    public static final int SHOOTER_FORWARD = 2;
    public static final int SHOOTER_REVERSE = 3;

    public static final int CONTROL_EXTEND = 0;
    public static final int CONTROL_RETRACT = 1;

    public static final int CLIMB_EXTEND = 6;
    public static final int CLIMB_RETRACT = 7;

    //LIMELIGHT
    
    public static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-bulls");
        
    public static final NetworkTableEntry tx = table.getEntry("tx");
    public static final NetworkTableEntry ty = table.getEntry("ty");
    public static final NetworkTableEntry ta = table.getEntry("ta");
    public static final NetworkTableEntry tv = table.getEntry("tv");
    

    public static final NetworkTableEntry ledMode = table.getEntry("ledMode");
    public static final NetworkTableEntry camMode = table.getEntry("camMode");
    public static final NetworkTableEntry pipeline = table.getEntry("pipeline");
    public static final NetworkTableEntry stream = table.getEntry("stream");

    //TRAJECTORY GENERATION
    
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    //Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

    //Horizontal distance between wheels --CHANGE
    public static final double kTrackwidthMeters = 0.69;

    //MAX ACCEL --CHANGE
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //LIMELIGHT DISTANCE CALCULATIONS -> ALL UNITS IN INCHES

    // Distance between limelight and ground
    public static final double limelightHeight = 21;
    // Distance between center of powerport and ground
    public static final double powerPortHeight = 50;
    // Angle of limelight - 30 degrees
    public static final int limelightAngle = 30;
    
}   
