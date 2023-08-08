package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.RobotContainer;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.subsystems.Elevator;


public class CmdManager {
    private static NAR_XboxController controller = RobotContainer.controller;
    private static Elevator elevator = Elevator.getInstance();



    private CmdManager() {}

    public static CommandBase vibrateController() {
        return new ScheduleCommand(new WaitCommand(0.5).deadlineWith(new StartEndCommand(() -> controller.startVibrate(), () -> RobotContainer.controller.stopVibrate())));
    }

    public static CommandBase moveElevator(double setpoint) {
        return sequence(
            runOnce(() -> elevator.startPID(setpoint), elevator),
            waitUntil(() -> elevator.atSetpoint())
        );
        // return run(() -> elevator.startPID(setpoint), elevator);
    }

    public static CommandBase moveElevator(Elevator.States setpoint) {
        return sequence(
            runOnce(() -> elevator.startPID(setpoint.height), elevator),
            waitUntil(() -> elevator.atSetpoint())
        );
    }
   

    


    
}