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

public class CmdManager {
    private static NAR_XboxController controller = RobotContainer.controller;
    private static Manipulator manipulator = Manipulator.getInstance();

    private CmdManager() {}

    public static CommandBase CmdManipIntakeCone() {
        return sequence(
            runOnce(() -> manipulator.intakeCone(), manipulator),
            waitUntil(() -> manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopRoller())
        );
    }

    public static CommandBase CmdManipIntakeCube() {
        return sequence(
            runOnce(() -> manipulator.intakeCube(), manipulator),
            waitUntil(() -> manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopRoller())
        );
    }

    public static CommandBase CmdManipStop() {
        return run(()-> manipulator.stopRoller());
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