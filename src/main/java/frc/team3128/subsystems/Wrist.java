package frc.team3128.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import static frc.team3128.Constants.WristConstants.*;

import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;

//Set up absolute encoder code to easily switch
public class Wrist extends NAR_PIDSubsystem {
    public NAR_CANSparkMax m_wrist;

    private static Wrist instance;

    public static Wrist getInstance() {
        if (instance == null){
            instance = new Wrist();  
        }
        return instance;
    }

    public Wrist() {
        super(new PIDController(kP, kI, kD), kS, kV, kG);
        setkG_Function(()-> Math.cos(Units.degreesToRadians(getMeasurement())));
        setConstraints(MIN_ANGLE, MAX_ANGLE);
        configMotor();
    }

    private void configMotor() {
        m_wrist = new NAR_CANSparkMax(WRIST_ID);
        m_wrist.setInverted(false);
        m_wrist.setIdleMode(IdleMode.kBrake);
        resetEncoder();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        m_wrist.set(MathUtil.clamp(output / 12.0, -1, 1));
    }

    @Override
    public double getMeasurement() {
        return m_wrist.getSelectedSensorPosition() * ROTATION_TO_DEGREES / GEAR_RATIO;
    }

    public void resetEncoder() {
        m_wrist.setSelectedSensorPosition(0);
    }

}
