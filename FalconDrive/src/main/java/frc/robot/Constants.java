/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    /**
     * constants for all the ports
     */
    public static final class PortConstants
    {
        public static final int lMainFalcon = 1;
        public static final int rMainFalcon = 3;
        public static final int lSubFalcon  = 2;
        public static final int rSubFalcon  = 4;

        public static final int intake    = 5;
        public static final int magazine  = 6;
        public static final int rFeeder   = 7;
        public static final int lFeeder   = 8;
        public static final int elevator1 = 9;
        public static final int elevator2 = 10;
        public static final int lShooter  = 11;
        public static final int rShooter  = 12;
        public static final int spinner   = 13;

        public static final int beamSensor   = 14;
        public static final int fPiston  = 5;
        public static final int rPiston  = 4;
    }
    /**
     * constants for vision tracking
     */
    public static class VisionConstants
    {
        public static final double kP = 0.0015;//.06
        public static final double kI = 0.001;

        public static final double minThreshold = .3; //error degrees horizontally
        public static final double maxSteer     = .4;
        public static final double minDriveSpeed = .26;
        public static final double offset = 3.8;//5.071236(NEAR) 3.198624(FAR);
        public static final double offsetNear = 5.071236;
        public static final double offsetFar  = 3.198624;

        public static final double towerHeight = 98;
        public static final double limeHeight = 21.5;
        public static final double limeAngle = 18.55362925;
    }
    /**
     * constants used for the autonomous part of the competition
     */
    public static class AutoConstants
    {
        public static final double ksVolts = .193;
        public static final double kvVoltSecondsPerMeter = 2.93;
        public static final double kaVoltSecondsSquaredPerMeter = 0.245;
        public static final double kPDriveVel = .00293;
        //possible 42, 0.00293, 0.492, 0.25
        public static final double kTrackwidthMeters = 0.641241342;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        //revolutions per count * wheel diameter (meters) / gear ratio
        public static final double distancePerPulse = (1.0 / 2048) * (.1524 * Math.PI) / 12.92;;
        public static final double driveDistance = 1;

        //deprecated maybe
        public static final double driveThreshold = 100.0;
    }
    /**
     * percent power for speed
     */
    public static class SpeedConstants
    {
        public static final double driveSpeed = 0.8;
        public static final double autoDriveSpeed = 0.4;

        public static final double minShootSpeed = .9;
        public static final double maxShootSpeed = 0.95;  //changed from 1.0
        //needs to be experimentally found
        public static final double minArea = 0;
        public static final double maxArea = 0.98;

        public static final double magazineSpeed = 0.4;
        public static final double intakeSpeed = 1.0; //changed from 0.8
        public static final double feederSpeed = -.9;
        public static final double elevatorSpeed = 0.5;
        public static final double spinnerSpeed = 0.5;

        public static final double rampSpeed = 0.5;
        public static final double driveRampSpeed = 0.25;
        public static final double autoDriveRampSpeed = 5.0;
    }

    public static class ShooterConstants
    {
	    public static final int kSlotIdx = 0;
	    public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
         * 
         * 	                                    			  kP   	 kI    kD      kF       Iz    PeakOut */
        public final static Gains kGains_Velocit  = new Gains( 0.2, 0.00001, 8, 1023.0/21264,  100,  1.00);
    }

    
}
