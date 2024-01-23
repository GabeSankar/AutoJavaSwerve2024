// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class constants {
    public static final double k_PI = 3.141592653589793238462643383279502884197169399;

    public static final double DrivingMotorGearRatio = (1.0/6.75);
    public static final double WheelDiameterInMeters = 0.0967;
    public static final double TurningMotorGearRatio = (7.0/150.0);
    public static final double k_DriveEncoderPosFactor = (DrivingMotorGearRatio * k_PI * WheelDiameterInMeters);
    public static final double k_DriveEncoderVelocityFactor = (k_DriveEncoderPosFactor / 60.0);
    public static final double k_TurnEncoderPosFactor = (TurningMotorGearRatio * k_PI * 2.0);
    public static final double k_TurnEncoderVelocityFactor = (k_TurnEncoderPosFactor / 60.0);

    public static final double k_KRP = .45;
    public static final double k_KRI = 0.0;
    public static final double k_KRD = 0.00025;

    public static final double k_maxSpeed = 4.441;
    public static final double k_maxAcc = 2.22;
    
    public static final double k_offsetadj = 1.0;

    public static final double MedAlpha =20.928478;
    public static final double MedBeta =-72.240211;
    public static final double MedGamma =-35.64251;
    
    public static final double StowAlpha =0.476190;
    public static final double StowBeta =-3.238093;
    public static final double StowGamma =-1.976190;
    
    public static final double AutoStowAlpha =0.476190;
    public static final double AutoStowBeta =-1.638093;
    public static final double AutoStowGamma =-1.976190;
    
    public static final double HIAlpha =35.213959;
    public static final double HIBeta =-160.423477;
    public static final double HIGamma =-45.809055;
    
    public static final double FloorAlpha =31.8;
    public static final double FloorBeta =-8.404788;
    public static final double FloorGamma =-86.479698;
    
    public static final double[] kTranslationalPIDGains = {2, 0, 0.002}; // {kP, kI, kD}
    public static final double[] kRotationalPIDGains = {3.25, 0, 0.008}; // {kP, kI, kD}
    
    public static final double k_VisionXTolerance = .10;
    public static final double k_VisionYTolerance = .10;
    public static final double k_VisionRotTolerance = .05;
    
    public static final double ShelfAlpha = 3.380949;
    public static final double ShelfBeta = -94.741562;
    public static final double ShelfGamma = -193.466797;
    // public static final double ShelfAlpha 8.500027
    // public static final double ShelfBeta -98.335808
    // public static final double ShelfGamma -178.277390
    
    public static final double AutoPlaceAlpha =2.0;
    public static final double AutoPlaceBeta =-232.842621;
    public static final double AutoPlaceGamma= -83.979454;
    // public static final double k_RBZERO = (3.204486-1.57079632) - k_PI + 0.1;
    // public static final double k_RFZERO = (1.761010-1.57079632)- k_PI;
    // public static final double k_LBZERO = (5.787710-1.57079632)- k_PI;
    // public static final double k_LFZERO = (5.514661-1.57079632)- k_PI;
    // public static final double k_RFZERO =(1.761010-1.57079632)-3.1415926535 + 0.009;
    // public static final double k_RBZERO =(3.204486-1.57079632)-3.1415926535 + 0.1 +0.023010;
    // public static final double k_LFZERO =(5.514661-1.57079632)-3.1415926535 -0.06;
    // public static final double k_LBZERO =(5.787710-1.57079632)-3.1415926535 + 0.006;
    //
    //Offset values for motor604388
    public static final double k_RFZERO =1.050776839701664;//1.055379;
    public static final double k_RBZERO =0.796136028912648-3.141592653;//0.777728-3.1415926535;
    public static final double k_LFZERO =1.810097329705057;//1.787;
    public static final double k_LBZERO =0.196349540849362-3.141592653;//0.199418-3.1415926535;

    public static final double k_gyroOffset = -k_PI;

    public static final double k_maxRotSpeed = 9.89;
    public static final double k_maxRotAcc = 4.5; // change to 9.89 after testing

    //Auto things
    //anglepid
    public static final double k_AAP = 9.3;
    public static final double k_AAI = 0.0;
    public static final double k_AAD = 0.0;
    //Constraint for Angled PID
        public static final TrapezoidProfile.Constraints k_angleControllerConstriants =
        new TrapezoidProfile.Constraints(
            k_maxRotSpeed, k_maxRotAcc);
    //xpid
    public static final double k_AXP = 3.14;
    public static final double k_AXI = 0.0;
    public static final double k_AXD = 0.0;
    //ypid
    public static final double k_AYP = 3.14;
    public static final double k_AYI = 0.0;
    public static final double k_AYD = 0.0;

}
