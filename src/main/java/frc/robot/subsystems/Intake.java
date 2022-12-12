package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;

public class Intake extends SubsystemBase {

    private final CANSparkMax m_motor;
    private double speed;

    public Intake() {
        m_motor = MotorFactory.createSparkMAX(Constants.intake.kMotorId, MotorType.kBrushless);
    }
    
    public void setMotor(double speed){
        this.speed=speed;
        m_motor.set(speed);
        System.out.printf("Intake motor set to %.2f\n", speed);
    }

    public double getSpeed(){
        return speed;
    }

    public void toggle(){
        if(getSpeed()==0){
            setMotor(1);
        }else{
            setMotor(0);
        }
    }
}
