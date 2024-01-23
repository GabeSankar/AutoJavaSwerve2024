package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.constants;

public class ArmCommand extends Command {
    private double alpha;
    private double beta;
    private double gamma;
    private double multiplier;
    private Arm m_robotArm;

    public ArmCommand(double alpha, double beta, double gamma, double multipier,Arm m_robotArm){
        this.alpha = alpha;
        this.beta = beta;
        this.gamma = gamma;
        this.multiplier = multiplier;
        this.m_robotArm = m_robotArm;
        addRequirements(m_robotArm);
    }
    @Override
    public void execute() {
       this.m_robotArm.GoTo(this.alpha,beta,gamma,this.multiplier);
        //SmartDashboard.putNumber("vision x pid setpoint", m_xPIDController.getSetpoint().position);
    }

    @Override
    public boolean isFinished(){
        if( Math.abs(alpha -m_robotArm.e_alpha.getPosition()) < 10
                                     && Math.abs(beta-m_robotArm.e_beta.getPosition()) < 10
                                     && Math.abs(gamma-m_robotArm.e_gamma.getPosition()) < 10)
            return true;
        else
            return false;
    }

}
