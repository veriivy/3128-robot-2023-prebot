// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import common.core.NAR_Robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.commands.CmdManager;
import frc.team3128.subsystems.Leds;
import frc.team3128.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends NAR_Robot  {
    public static Robot instance;

    public static RobotContainer m_robotContainer = new RobotContainer();
    public static AutoPrograms autoPrograms = new AutoPrograms();
    public Timer xLockTimer = new Timer();
    public double startTime;

    public static synchronized Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    @Override
    public void robotInit(){
        new Trigger(this::isEnabled).negate().debounce(2).onTrue(new InstantCommand(()-> Swerve.getInstance().setBrakeMode(false)).ignoringDisable(true));
        m_robotContainer.init();
        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotPeriodic(){
        m_robotContainer.updateDashboard();        
    }

    @Override
    public void autonomousInit() {
        xLockTimer.restart();
        Command m_autonomousCommand = autoPrograms.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
        if (xLockTimer.hasElapsed(14.75)) {
            new RunCommand(()->Swerve.getInstance().xlock(), Swerve.getInstance()).schedule();
        }
    }

    @Override
    public void teleopInit() {
        Leds.getInstance().defaultColor = CmdManager.SINGLE_STATION ? Colors.CHUTE : Colors.SHELF;
        Leds.getInstance().resetLeds();
        xLockTimer.restart();
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        if (xLockTimer.hasElapsed(134.75)) {
            // new RunCommand(()-> Swerve.getInstance().xlock(), Swerve.getInstance()).schedule();
        }
    }

    @Override
    public void simulationInit() {
        
    }

    @Override
    public void simulationPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        Swerve.getInstance().setBrakeMode(true);
    }
    
    @Override
    public void disabledPeriodic() {

    }
}
