package frc.team3128.subsystems;

import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.team3128.Constants.WristConstants.*;

import frc.team3128.RobotContainer;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.utility.NAR_Shuffleboard;

public class Wrist extends NAR_PIDSubsystem {
    public NAR_CANSparkMax m_wrist;

    private static Wrist instance;

    public Wrist() {
        super(new PIDController(kP, kI, kD), kS, kV, kG, kG -> (Double)(Math.sin(kG)));
        setConstraints(MIN_ANGLE, MAX_ANGLE);
        configMotor();
    }

    public static Wrist getInstance() {
        if (instance == null){
            instance = new Wrist();  
        }
        
        return instance;
    }

    public void configMotor() {
        m_wrist = new NAR_CANSparkMax(WRIST_ID);
        m_wrist.setInverted(false);
        m_wrist.setIdleMode(IdleMode.kBrake);
        m_wrist.enableVoltageCompensation(12.0);
        resetEncoder();
    }

    public double getAngle() {
        return m_wrist.getSelectedSensorPosition() * ROTATION_TO_DEGREES / GEAR_RATIO;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        m_wrist.set(MathUtil.clamp(output / 12.0, -1, 1));
    }

    @Override
    protected double getMeasurement() {
        return getAngle();
    }

    public void resetEncoder() {
        m_wrist.setSelectedSensorPosition(0);
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData("Wrist", "Wrist Angle", () -> getAngle(), 0, 1);
    }

}
