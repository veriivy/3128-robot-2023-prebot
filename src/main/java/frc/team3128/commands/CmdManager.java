package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.RobotContainer;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Led;
import frc.team3128.subsystems.Wrist;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Vision;

public class CmdManager {
    private static Led led = Led.getInstance();
    private static Wrist wrist = Wrist.getInstance();
    private static Manipulator manipulator = Manipulator.getInstance();
    private static NAR_XboxController controller = RobotContainer.controller;


    private CmdManager() {}

    public static CommandBase CMDWrist() {
        return new InstantCommand(()-> wrist.);
    }

    public static CommandBase ManipIntake(Boolean cone) {
        return new InstantCommand(()-> manipulator.intake(cone));
    }

    public static CommandBase ManipOuttake() {
        return new InstantCommand(()-> manipulator.outtake());
    }

    public static CommandBase vibrateController() {
        return new ScheduleCommand(new WaitCommand(0.5).deadlineWith(new StartEndCommand(() -> controller.startVibrate(), () -> RobotContainer.controller.stopVibrate())));
    }
}