package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Arm extends SubsystemBase{
    private CANSparkMax m_alpha1;
    private CANSparkMax m_alpha2;
    private CANSparkMax m_beta;
    private CANSparkMax m_gamma;
    private CANSparkMax m_claw;

    private PIDController pid_alpha = new PIDController(.1, 0, 0);
    private PIDController pid_beta = new PIDController(.1, 0, 0);
    private PIDController pid_gamma = new PIDController(.1, 0, 0);
    public RelativeEncoder e_alpha;
    public RelativeEncoder e_beta;
    public RelativeEncoder e_gamma;
    public Arm(int m_alpha1, int m_alpha2, int m_beta, int m_gamma, int m_claw){
        this.m_alpha1 = new CANSparkMax(m_alpha1,MotorType.kBrushless);
        this.m_alpha2 = new CANSparkMax(m_alpha2,MotorType.kBrushless);
        this.m_beta = new CANSparkMax(m_beta,MotorType.kBrushless);
        this.m_gamma = new CANSparkMax(m_gamma,MotorType.kBrushless);
        this.m_claw = new CANSparkMax(m_claw,MotorType.kBrushless);

        this.e_alpha = this.m_alpha1.getEncoder();
        this.e_beta = this.m_beta.getEncoder();
        this.e_gamma = this.m_gamma.getEncoder();

        this.zeroEncoders();
      
    }
    public void zeroEncoders(){
        e_alpha.setPosition(0.0);
        e_beta.setPosition(0.0);
        e_gamma.setPosition(0.0);
    }
    public void SetClaw(double power){
        this.m_claw.set(power);
    }
    public void DriveStick(Joystick alpha,Joystick beta){
        double joint1CloseToBatteryMotorSpeed = MathUtil.applyDeadband(alpha.getY(),.4)*.3;
        double joint1AwayFromBatteryMotorSpeed = -MathUtil.applyDeadband(alpha.getY(),.4)*.3;
        double joint2MotorSpeed = MathUtil.applyDeadband(beta.getY(),.4)*.5;

        this.m_alpha1.set(joint1AwayFromBatteryMotorSpeed);
        this.m_alpha2.set(joint1CloseToBatteryMotorSpeed);
        this.m_beta.set(joint2MotorSpeed);

        if(alpha.getRawButton(2))
            this.m_gamma.set(-0.3);
        else if(alpha.getRawButton(3))
            this.m_gamma.set(.3);
        else
            this.m_gamma.set(0.0);

    }
    
    public void GoTo(double Alpha, double Beta, double Gamma, double multiplier){
        double alphapos = e_alpha.getPosition();
        double betapos = e_beta.getPosition();
        double gammapos = e_gamma.getPosition();

        double Alpha1Speed = pid_alpha.calculate(alphapos,Alpha);
        double Alpha2Speed = -pid_alpha.calculate(alphapos,Alpha);
        double BetaSpeed = pid_beta.calculate(betapos,Beta);
        double GammaSpeed = pid_gamma.calculate(gammapos,Gamma);

        if(Alpha - alphapos > 5.0){
            m_alpha1.set(.15*multiplier); // Alpha Forward
            m_alpha2.set(-.15*multiplier); //Alpha Forward
            }else if(Alpha - alphapos < -5.0){
            m_alpha1.set(-.25*multiplier); // Alpha Backward
            m_alpha2.set(.25*multiplier); // Alpha Backward
            }else{
            m_alpha1.set(Alpha1Speed);
            m_alpha2.set(Alpha2Speed);
            }

            if(Beta - betapos > 5.0){
                m_beta.set(.7*multiplier); // Beta backward
                }else if(Beta - betapos < -5.0){
                m_beta.set(-1.0*multiplier); //Beta forward
                }else{
                m_beta.set(BetaSpeed);
                }
            
            if(Gamma - gammapos > 10.0){
                m_gamma.set(.4*multiplier);
                }else if(Gamma - gammapos < -10.0){
                m_gamma.set(-.4*multiplier);
                }else{
                m_gamma.set(GammaSpeed);
                }
    }
}
/* 


void Arm::SetClawSpinner(double power){
   //m_clawSpinner.Set(power); 
}


void Arm::ResetEncoders(){
    e_alpha -> SetPosition(0.0);
    e_beta -> SetPosition(0.0);
    e_gamma -> SetPosition(0.0);

}
void Arm::GoTo(double Alpha, double Beta, double Gamma, double multiplier){ // Multiplier is a double from 0.0 to 1.0

    double alphapos = e_alpha -> GetPosition();
    double betapos = e_beta -> GetPosition();
    double gammapos = e_gamma -> GetPosition();

    double Alpha1Speed = pid_alpha.Calculate(alphapos, Alpha);
    double Alpha2Speed = -pid_alpha.Calculate(alphapos, Alpha);
    double BetaSpeed = pid_beta.Calculate(betapos, Beta);
    double GammaSpeed = pid_gamma.Calculate(gammapos,Gamma);

    if(Alpha - alphapos > 5.0){
    m_alphaMotor1.Set(.15*multiplier); // Alpha Forward
    m_alphaMotor2.Set(-.15*multiplier); //Alpha Forward
    }else if(Alpha - alphapos < -5.0){
    m_alphaMotor1.Set(-.25*multiplier); // Alpha Backward
    m_alphaMotor2.Set(.25*multiplier); // Alpha Backward
    }else{
    m_alphaMotor1.Set(Alpha1Speed);
    m_alphaMotor2.Set(Alpha2Speed);
    }
    
    if(Beta - betapos > 5.0){
    m_betaMotor.Set(.7*multiplier); // Beta backward
    }else if(Beta - betapos < -5.0){
    m_betaMotor.Set(-1.0*multiplier); //Beta forward
    }else{
    m_betaMotor.Set(BetaSpeed);
    }

    if(Gamma - gammapos > 10.0){
    m_gammaMotor.Set(GammaMultiplier*multiplier);
    }else if(Gamma - gammapos < -10.0){
    m_gammaMotor.Set(-GammaMultiplier*multiplier);
    }else{
    m_gammaMotor.Set(GammaSpeed);
    }

}

void Arm::GoToAuto(double Alpha, double Beta, double Gamma, double multiplier){ // Multiplier is a double from 0.0 to 1.0

    double alphapos = e_alpha -> GetPosition();
    double betapos = e_beta -> GetPosition();
    double gammapos = e_gamma -> GetPosition();

    double Alpha1Speed = pid_alpha.Calculate(alphapos, Alpha);
    double Alpha2Speed = -pid_alpha.Calculate(alphapos, Alpha);
    double BetaSpeed = pid_beta.Calculate(betapos, Beta);
    double GammaSpeed = pid_gamma.Calculate(gammapos,Gamma);

    if(Alpha - alphapos > 5.0){
    m_alphaMotor1.Set(.15*multiplier); // Alpha Forward
    m_alphaMotor2.Set(-.15*multiplier); //Alpha Forward
    }else if(Alpha - alphapos < -5.0){
    m_alphaMotor1.Set(-.25*multiplier); // Alpha Backward
    m_alphaMotor2.Set(.25*multiplier); // Alpha Backward
    }else{
    m_alphaMotor1.Set(Alpha1Speed);
    m_alphaMotor2.Set(Alpha2Speed);
    }
    
    if(Beta - betapos > 5.0){
    m_betaMotor.Set(.7*multiplier); // Beta backward
    }else if(Beta - betapos < -5.0){
    m_betaMotor.Set(-1.0*multiplier); //Beta forward
    }else{
    m_betaMotor.Set(BetaSpeed);
    }

    if(Gamma - gammapos > 10.0){
    m_gammaMotor.Set(0.4*multiplier);
    }else if(Gamma - gammapos < -10.0){
    m_gammaMotor.Set(-0.4*multiplier);
    }else{
    m_gammaMotor.Set(GammaSpeed);
    }

} */