package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;
import edu.wpi.first.math.controller.PIDController;


public class Limelight extends SubsystemBase {
    private final String m_name;
    private double[] m_botpose = new double[6];

    public Limelight(String name) {
        m_name = name;
    }

    @Override
    public void periodic() {
        m_botpose = NetworkTableInstance.getDefault().getTable(m_name).getEntry("botpose").getDoubleArray(new double[6]);
    }

    public boolean hasTarget() {
        return NetworkTableInstance.getDefault().getTable(m_name).getEntry("tv").getDouble(0) == 1;
    }

    public double getBotX() {
        return m_botpose[0];
    }
    
    public double getBotY() {
        return m_botpose[1];
    }

    public double getBotZ() {
        return m_botpose[2];
    }

    public double getBotRoll() {
        return m_botpose[3];
    }

    public double getBotPitch() {
        return m_botpose[4];
    }

    public double getBotYaw() {
        return m_botpose[5];
    }
}