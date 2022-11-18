package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;

public class Intake extends SubsystemBase {

    private final CANSparkMax m_motor;

    public Intake() {
        m_motor = MotorFactory.createSparkMAX(Constants.intake.kMotorId, MotorType.kBrushless);
    }
    
    
}
