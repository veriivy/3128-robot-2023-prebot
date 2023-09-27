package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import static frc.team3128.Constants.ManipulatorConstants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Manipulator extends SubsystemBase {
    private NAR_TalonSRX m_roller;

    private static Manipulator instance;

    private boolean outtaking = false;

    private boolean cone = false;

    private Manipulator() {
        configMotor();
    }

    public static synchronized Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }
        return instance;
    }

    @Override
    public void periodic() {
        // if (hasObjectPresent() && !outtaking) {
        //     stopRoller();
        // }
    }

    public void configMotor() {
        m_roller = new NAR_TalonSRX(ROLLER_MOTOR_ID);
        m_roller.setNeutralMode(NeutralMode.Brake);
        m_roller.setInverted(true);
    }

    public void intakeCone() {
        cone = true;
        outtaking = false;
        m_roller.set(CONE_ROLLER_POWER);
    }

    public void intakeCube() {
        cone = false;
        outtaking = false;
        m_roller.set(CUBE_ROLLER_POWER);
    }

    public void outtake() {
        outtaking = true;
        m_roller.set(OUTTAKE_POWER);
    }

    public void shoot() {
        outtaking = true;
        m_roller.set(SHOOT_POWER);
    }

    public void stopRoller() {
        m_roller.set(0);
    }

    public boolean hasObjectPresent() {
        return Math.abs(m_roller.getStatorCurrent()) > CURRENT_THRESHOLD;
    }
}
