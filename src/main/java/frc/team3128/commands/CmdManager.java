package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.RobotContainer;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Wrist;

public class CmdManager {
    private static NAR_XboxController controller = RobotContainer.controller;
    private static Wrist wrist = Wrist.getInstance();
    private static Manipulator manipulator = Manipulator.getInstance();

    private CmdManager() {}

    public static CommandBase CmdWrist(double angle) {
        return sequence(
            runOnce(() -> wrist.startPID(angle), wrist),
            waitUntil(()-> wrist.atSetpoint())
        );
    }

    public static CommandBase CmdManipIntake() {
        return sequence(
            runOnce(() -> manipulator.intake(), manipulator),
            waitUntil(() -> manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopRoller())
        );
    }

    public static CommandBase CmdManipOutake() {
        return sequence(
            runOnce(() -> manipulator.outtake(), manipulator),
            waitUntil(() -> !manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopRoller())
        );
    }

    public static CommandBase CmdManipShoot() {
        return sequence(
            runOnce(() -> manipulator.shoot(), manipulator),
            waitUntil(() -> !manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopRoller())
        );
    }
    public static CommandBase vibrateController() {
        return new ScheduleCommand(new WaitCommand(0.5).deadlineWith(new StartEndCommand(() -> controller.startVibrate(), () -> RobotContainer.controller.stopVibrate())));
    }
}