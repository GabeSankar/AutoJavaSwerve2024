package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.constants;
public class VisionTargetCommand extends Command {
    
    private final Drivetrain m_drive;
    private final double m_xGoal;
    private final double m_yGoal;
    private final double m_rotGoal;
    private double m_xPos;
    private double m_yPos;
    private double m_rotPos;

    private final PIDController m_xPIDController = new PIDController(
        constants.kTranslationalPIDGains[0],
        constants.kTranslationalPIDGains[1],
        constants.kTranslationalPIDGains[2]
    );

    private final PIDController m_yPIDController = new PIDController(
        constants.kTranslationalPIDGains[0],
        constants.kTranslationalPIDGains[1],
        constants.kTranslationalPIDGains[2]
    );

    private final PIDController m_rotPIDController = new PIDController(
        constants.kRotationalPIDGains[0],
        constants.kRotationalPIDGains[1],
        constants.kRotationalPIDGains[2]
    );

    private final LinearFilter m_xFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter m_yFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter m_rotFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private double s = 1.0;
    private double m_xprior =1000.0;
    private double m_yprior = 1000.0;
    private double m_rotprior = 1000.0;
    public VisionTargetCommand(Drivetrain drive, double xGoal, double yGoal, double rotGoal) {
        m_drive = drive;
        m_xGoal = xGoal;
        m_yGoal = yGoal;
        m_rotGoal = rotGoal;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        /*m_xPIDController.reset(m_drive.getVisionX());
        m_yPIDController.reset(m_drive.getVisionY());
        m_rotPIDController.reset(m_drive.getVisionYaw());
        m_xPIDController.setGoal(m_xGoal);
        m_yPIDController.setGoal(m_yGoal);/
        m_rotPIDController.setGoal(m_rotGoal);*/
        m_xprior = 1000.0;
        m_yprior = 1000.0;
        m_rotprior = 1000.0;
        m_xPIDController.setSetpoint(m_xGoal);
        m_yPIDController.setSetpoint(m_yGoal);
        m_rotPIDController.setSetpoint(m_rotGoal);
        m_xFilter.reset();
        m_yFilter.reset();
        m_rotFilter.reset();
    }

    @Override
    public void execute() {
        if(m_drive.visionHasTarget()){
            if(m_xprior != 1000.0){
                m_xPos= m_xprior+(m_drive.getVisionX()-m_xprior);
                m_yPos = m_yprior+ (m_drive.getVisionY()-m_yprior);
                m_rotPos = m_rotprior + (m_drive.getVisionYaw()/180*Math.PI -m_rotprior);
            }
            else{
                m_xPos = m_drive.getVisionX();
                m_yPos = m_drive.getVisionY();
                m_rotPos = m_drive.getVisionYaw()/180*Math.PI;
            }

            // if (Math.abs(m_xPos-m_xGoal) < constants.k_VisionXTolerance) m_xPos = m_xGoal;
            // if (Math.abs(m_yPos-m_yGoal) < constants.k_VisionYTolerance) m_yPos = m_yGoal;
            // if (Math.abs(m_rotPos-m_rotGoal) < constants.k_VisionRotTolerance) m_rotPos = m_rotGoal
            // Note: here the field relative is set to false because it allows the robot to
            // center on the 16h5 id 2 tag on the 2023 field independent of the gyro readings
            // (basically you can move the tag). On the actual field, field relative must be
            m_xprior = m_drive.getVisionX();
            m_yprior = m_drive.getVisionY();
            m_rotprior = m_drive.getVisionYaw()/180*Math.PI;
            m_drive.Drive(
                MathUtil.clamp(m_xPIDController.calculate(m_xPos), -constants.k_maxSpeed, constants.k_maxSpeed),/*+m_xPIDController.getSetpoint().velocity*/
                MathUtil.clamp(m_yPIDController.calculate(m_yPos), -constants.k_maxSpeed, constants.k_maxSpeed),/*+m_yPIDController.getSetpoint().velocity*/
                MathUtil.clamp(m_rotPIDController.calculate(m_rotPos), -constants.k_maxRotSpeed, constants.k_maxRotSpeed),/*+m_rotPIDController.getSetpoint().velocity*/
                false);
                ///was originally robot relative
        }else{
            m_drive.Drive(0.0,0.0,0.0, false);
        }
    
        //SmartDashboard.putNumber("vision x pid setpoint", m_xPIDController.getSetpoint().position);
    }
    @Override
    public void end(boolean interrupted){
        m_drive.Drive(0.0,0.0,0.0, false);
    }
    @Override
    public boolean isFinished(){
        if( Math.abs(m_xPos -m_xGoal) < constants.k_VisionXTolerance
                                     && Math.abs(m_yPos-m_yGoal) < constants.k_VisionYTolerance
                                     && Math.abs(m_rotPos-m_rotGoal) < constants.k_VisionRotTolerance)
            return true;
        else
            return false;
    }

}