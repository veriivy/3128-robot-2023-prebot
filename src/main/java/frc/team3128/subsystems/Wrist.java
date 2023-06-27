package frc.team3128.subsystems;

import java.util.function.DoubleFunction;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.team3128.Constants.WristConstants.*;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;

public class Wrist extends NAR_PIDSubsystem {
    public NAR_CANSparkMax m_pivot;

    private static Wrist instance;

    public Wrist() {
        super(new PIDController(kP, kI, kD), kS, kV, kG, kG -> (Double)(Math.sin(kG)));
        configMotor();
    }

    public static Wrist getInstance() {
        if (instance == null){
            instance = new Wrist();  
        }
        
        return instance;
    }

    public void configMotor() {
        m_pivot = new NAR_CANSparkMax(PIVOT_ID);
        m_pivot.setInverted(false);
        m_pivot.setIdleMode(IdleMode.kBrake);
        m_pivot.enableVoltageCompensation(12.0);
        resetEncoder();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        m_pivot.set(MathUtil.clamp(output / 12.0, -1, 1));
    }

    @Override
    protected double getMeasurement() {
        return m_pivot.getSelectedSensorPosition() * ROTATION_TO_DEGREES / GEAR_RATIO;
    }

    public void resetEncoder() {
        m_pivot.setSelectedSensorPosition(0);
    }




}
