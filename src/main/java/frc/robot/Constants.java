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
        
        public static final int bLiftRun = Button.kA.value;
        public static final int bWinchRun = Button.kB.value;
        public static final int bWinchBack = Button.kY.value;
        public static final int bLiftback = Button.kX.value;
        public static final int bSecondLiftUp = Button.kRightBumper.value;
        public static final int bSecondLiftDown = Button.kLeftBumper.value;

    }
    public static final class OperatorButtons{
        public static final int bFlapperup = Button.kRightBumper.value;
        public static final int bFlapperdown = Button.kLeftBumper.value;
        public static final int bArmUp = Button.kB.value;
        public static final int bArmDown = Button.kA.value;
        public static final int bIntakeRun = Button.kX.value;
        public static final int bTurn = Button.kY.value;
        public static final int bHalfSpeed = Button.kLeftBumper.value;
        public static final int bFullSpeed = Button.kRightBumper.value;
    }
    
    public static final class DriveConstants{
        public static final int aLeftDrivePort1 = 1;
        public static final int aLeftDrivePort2 = 2;
        public static final int aRightDrivePort1 = 4;
        public static final int aRightDrivePort2 = 5;

        public static final double aTrackwidthMeters = 0.7115; //0.48; //1.1;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(aTrackwidthMeters);

        public static final double aGearRatio = 10.93;
        public static final int aEncoderCPR = 2048;
        public static final double aWheelDiameterMeters = 0.1524;
        public static final double aEncoderDistancePerPulse = 
            //(aWheelDiameterMeters * Math.PI) / (double) aEncoderCPR;
            (aWheelDiameterMeters * Math.PI) /aGearRatio/ (double) aEncoderCPR;

        public static final double Conversion = 
            aWheelDiameterMeters * Math.PI /
                (aGearRatio * aEncoderCPR);

        public static final double asVolts = 0.49819; //0.61601; //0.60903;
        public static final double avVoltSecondsPerMeter = 2.38; //2.443; //0.062107;
        public static final double aaVoltSecondsSquaredPerMeter = 0.59487; //0.34883; //0.0087418;

        public static final double aPDriveVel = 2.6116; //19.916; //2.4169; //maybe 3.3387; //0.0044096; //0.084576;

        // Falcon Tolerance

        public static final int aAmpLimit = 30;
        public static final int aTriggerThreshold = 25;
        public static final double aTriggerTime = 1.0;

        // Gyro Turn Auto
        public static final double kTurnP = 0.062614;
        public static final double kTurnD = 0.0;
        public static final double kTurnI = 0.0;
        public static final double kTurnToleranceDeg = 1.0;
        public static final double kTurnRateToleranceDegPerS = 1.0;

    }

    public static final class AutoConstants{
        public static final double kAutoDriveTime = 1.5;
        public static final double kAutoDriveSpeed = -0.5;

        public static final double kAutoTurnSpeed = 0.45;
        public static final double kAutoSlowTurnSpeed = 0.3;

        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Auto Distances
        public static final double shootHighDistance = 2.5;
        public static final int shootLowDistance = 3;
        // Auto Transition Time
        public static final int transitionTime = 3;
    }

    public static final class MechConstants{
        //shooter subsystem
        public static final int aShooterPort1 = 3;
                //intake subsystem
        public static final int aRollerPort = 8;
        public static final int aArmPort = 6;
                //transition subsystem
        public static final int aTransitionPort = 5;
                //lift
        public static final int aLiftPort = 9;
                //winch
        public static final int aWinchPort = 10;
                //switches
        public static final int aArmDownSwitch = 0;
        public static final int aArmUpSwitch = 1;

        public static final int aLiftSwitchPort = 4;

        public static final int aSecondLiftPort = 11;
    }

    public static final class OIConstants {
        public static final int aDriverControllerPort = 0;
        public static final int aOperatorControllerPort = 1;
    }
    public static final class SpeedConstants{
        public static final double MturnSpeed = 0.60000001;
        public static final double driveSpeed = 0.9;
        public static final double aArmSpeed = 1.0;
        public static final double aRollerSpeed = 0.7;
        public static final double aTransitionSpeed = 0.75;
        public static final double aHighShootSpeed = 0.37;
        public static final double aLowShootSpeed = 0.475; // actually another high speed
        public static final double aHighCloseShootSpeed = 0.385;//0.37;
        public static final double kFarShootSpeed = 0.47;
        public static final double aLiftSpeed = 0.85;
        public static final double aWinchSpeed = 1.0;
        public static final double aTransitionSpeedAuto = 1.0;
        public static final double aSecondLiftSpeed = 0.5;
    }

}
