// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriverButtons{
        public static final int bArmUp = Button.kB.value;
        public static final int bArmDown = Button.kA.value;
        public static final int bTransitionRun = Button.kY.value;
        public static final int bShooterRun = Button.kRightBumper.value;
        public static final int bIntakeRun = Button.kX.value;
        public static final int bFlapperRun = Button.kLeftBumper.value;
        public static final int bLiftRun = Button.kRightBumper.value;
        public static final int bWinchRun = Button.kLeftBumper.value;
        public static final int bShooterRunOperator = Button.kY.value;
        
    }
    
    public static final class DriveConstants{
        public static final int aLeftDrivePort1 = 1;
        public static final int aLeftDrivePort2 = 2;
        public static final int aRightDrivePort1 = 4;
        public static final int aRightDrivePort2 = 5;

        public static final double aTrackwidthMeters = 0.24; //1.1;
        public static final DifferentialDriveKinematics aDriveKinematics =
            new DifferentialDriveKinematics(aTrackwidthMeters);

        public static final double aGearRatio = 10.93;
        public static final int aEncoderCPR = 2040;
        public static final double aWheelDiameterMeters = 0.16;
        public static final double aEncoderDistancePerPulse = 
            (aWheelDiameterMeters * Math.PI) / (double) aEncoderCPR;
        public static final double Conversion = 
            aWheelDiameterMeters * Math.PI /
                (aGearRatio * aEncoderCPR);

        public static final double asVolts = 1.01;
        public static final double avVoltSecondsPerMeter = 0.0952;
        public static final double aaVoltSecondsSquaredPerMeter = 0.0201;

        public static final double aPDriveVel = 0.913;

        public static final int aAmpLimit = 30;
        public static final int aTriggerThreshold = 25;
        public static final double aTriggerTime = 1.0;
    }

    public static final class AutoConstants{
        public static final double kAutoDriveTime = 3;
        public static final double kAutoDriveSpeed = -0.5;
    }

    public static final class MechConstants{
        //shooter subsystem
        public static final int aShooterPort1 = 3;
                //intake subsystem
        public static final int aRollerPort = 8;
        public static final int aArmPort = 6;
                //transition subsystem
        public static final int aTransitionPort = 5;
                //flabber
        public static final int aFlapper = 7;
                //lift
        public static final int aLiftPort = 9;
                //winch
        public static final int aWinchPort = 10;
                //switches
        public static final int aArmDownSwitch = 0;
        public static final int aArmUpSwitch = 1;
    }

    public static final class OIConstants {
        public static final int aDriverControllerPort = 0;
        public static final int aOperatorControllerPort = 1;
    }
    public static final class SpeedConstants{
        public static final double turnSpeed = 0.6;
        public static double driveSpeed = 0.8;
        public static final double aArmSpeed = 0.5;
        public static final double aRollerSpeed = 0.7;
        public static final double aTransitionSpeed = 0.5;
        public static final double aHighShootSpeed = 0.36;
        public static final double aLowShootSpeed = 0.4;
        public static final double aFlabberSpeed = 0.4;
        public static final double aLiftSpeed = 0.75;
        public static final double aWinchSpeed = 1.0;
    }

}
