package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;

public class Outtake {
    
    private final CANSparkMax m_motor;

    public Outtake() {
        m_motor = MotorFactory.createSparkMAX(Constants.outtake.kMotorId, MotorType.kBrushless);
    }
}
