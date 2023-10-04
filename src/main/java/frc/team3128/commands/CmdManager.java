package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.RobotContainer;
import frc.team3128.PositionConstants.Position;
import frc.team3128.common.hardware.input.NAR_XboxController;

import frc.team3128.subsystems.Led;
import frc.team3128.subsystems.ManipCRX;
import frc.team3128.subsystems.Wrist;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Elevator;

public class CmdManager {
    private static Led led = Led.getInstance();
    private static Wrist wrist = Wrist.getInstance();
    // private static Manipulator manipulator = Manipulator.getInstance();
    private static ManipCRX manipulator = ManipCRX.getInstance();
    private static Swerve swerve = Swerve.getInstance();
    private static NAR_XboxController controller = RobotContainer.controller;
    private static Elevator elevator = Elevator.getInstance();

    public static boolean ENABLE = false;
    public static boolean SINGLE_STATION = true;

    private CmdManager() {}

    public static CommandBase score(Position position, int xPos, boolean runImmediately) {
        return sequence(
            runOnce(()-> ENABLE = runImmediately),
            waitUntil(()-> ENABLE),
            extend(position),
            waitUntil(()-> !ENABLE),
            CmdManipOutake(),
            waitSeconds(0.5),
            CmdManipStop(),
            retract(Position.NEUTRAL)
        );
    }

    public static CommandBase score(Position position, int xPos) {
        return score(position, xPos, false);
    }

    public static CommandBase HPpickup(Position position1, Position position2) {
        return either(pickup(position1), pickup(position2), ()-> SINGLE_STATION);
    }

    public static CommandBase pickup(Position position, boolean runImmediately) {
        return sequence(
            runOnce(()-> ENABLE = runImmediately),
            waitUntil(()-> ENABLE),
            extend(position),
            waitSeconds(0.2),
            CmdManipIntake(position.cone),
            retract(Position.NEUTRAL)
        );
    }

    public static CommandBase pickup(Position position) {
        return pickup(position, false);
    }

    public static CommandBase extend(Position position) {
        return sequence (
            moveElevator(position),
            moveWrist(position)
        );
    }

    public static CommandBase retract(Position position) {
        return sequence (
            moveWrist(position),
            moveElevator(position)
        );
    }

    public static CommandBase moveWrist(double setpoint) {
        return sequence(
            runOnce(()-> wrist.startPID(setpoint), wrist),
            waitUntil(()-> wrist.atSetpoint())
        );
    }

    public static CommandBase moveWrist(Position position) {
        return sequence(
            runOnce(()-> wrist.startPID(position.wristAngle), wrist),
            waitUntil(()-> wrist.atSetpoint())
        );
    }

    public static CommandBase CmdManipIntake(boolean cone) {
        return sequence(
            runOnce(() -> manipulator.intake(cone), manipulator),
            waitSeconds(0.4),
            waitUntil(() -> manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopRoller(), manipulator)
        );
    }

    public static CommandBase CmdManipStop() {
        return runOnce(()-> manipulator.stopRoller(), manipulator);
    }

    public static CommandBase CmdManipOutake() {
        return sequence(
            runOnce(() -> manipulator.outtake(), manipulator),
            waitSeconds(0.4),
            waitUntil(() -> !manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopRoller(), manipulator)
        );
    }

    public static CommandBase CmdManipShoot() {
        return sequence(
            runOnce(() -> manipulator.shoot(), manipulator),
            waitUntil(() -> !manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopRoller(), manipulator)
        );
    }

    // public static CommandBase intake(Boolean cone) {
    //     return sequence(
    //         runOnce(()-> manipulator.intake(cone), manipulator),
    //         waitSeconds(0.4),
    //         waitUntil(()-> manipulator.hasObjectPresent()),
    //         waitSeconds(cone ? 0.15 : 0),
    //         runOnce(()-> manipulator.stallPower(), manipulator)
    //     );
    // }

    // public static CommandBase outtake() {
    //     return runOnce(()-> manipulator.outtake(), manipulator);
    // }

    // public static CommandBase stopManip() {
    //     return runOnce(()-> manipulator.stopRoller(), manipulator);
    // }

    public static CommandBase vibrateController() {
        return new ScheduleCommand(new WaitCommand(0.5).deadlineWith(new StartEndCommand(() -> controller.startVibrate(), () -> RobotContainer.controller.stopVibrate())));
    }

    public static CommandBase moveElevator(double setpoint) {
        return sequence(
            runOnce(() -> elevator.startPID(setpoint), elevator),
            waitUntil(() -> elevator.atSetpoint())
        );
    }

    public static CommandBase moveElevator(Position setpoint) {
        return sequence(
            runOnce(() -> elevator.startPID(setpoint.elvDist), elevator),
            waitUntil(() -> elevator.atSetpoint())
        );
    }

    public static CommandBase moveElv(double speed) {
        return runOnce(()-> elevator.set(speed), elevator);
    }

    public static CommandBase moveWri(double speed) {
        return runOnce(()-> wrist.set(speed), wrist);
    }

    public static CommandBase resetElevator() {
        return runOnce(()-> elevator.resetEncoder());
    }

    public static CommandBase resetWrist() {
        return runOnce(()-> wrist.resetEncoder());
    }

    public static CommandBase resetSwerve() {
        return runOnce(()-> swerve.zeroGyro());
    }
}