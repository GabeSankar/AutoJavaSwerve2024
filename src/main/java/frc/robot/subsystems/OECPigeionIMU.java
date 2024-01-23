package frc.robot.subsystems;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

public class OECPigeionIMU {
    PigeonIMU pigeonGyro;
    public OECPigeionIMU(int canPort){
        pigeonGyro = new PigeonIMU(canPort);
    }
    public void BootCalibrate(){
        pigeonGyro.enterCalibrationMode(CalibrationMode.BootTareGyroAccel, 200);
    }
    public void ResetYaw(){
        pigeonGyro.setYaw(0.0);
    }
    public double GetYaw(){
        double ypr[] = new double[3];
        pigeonGyro.getYawPitchRoll(ypr);
        return ypr[0];
    }
    public double GetPitch(){
        double ypr[] = new double[3];
        pigeonGyro.getYawPitchRoll(ypr);
        return ypr[1];
    }
    public double GetRoll(){
        double ypr[] = new double[3];
        pigeonGyro.getYawPitchRoll(ypr);
        return ypr[2];
    }

}
