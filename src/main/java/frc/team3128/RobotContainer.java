package frc.team3128;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.commands.CmdSwerveDrive;

import static frc.team3128.Constants.FieldConstants.*;

import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.PositionConstants.Position;
import frc.team3128.commands.CmdBalance;
import frc.team3128.commands.CmdBangBangBalance;
import static frc.team3128.commands.CmdManager.*;
import frc.team3128.common.hardware.input.NAR_ButtonBoard;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Elevator;
import frc.team3128.subsystems.Led;
import frc.team3128.subsystems.ManipCRX;
import frc.team3128.common.utility.NAR_Shuffleboard;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

import java.util.function.BooleanSupplier;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;
    private Vision vision;
    private Led led;
    private Elevator elevator;
    private ManipCRX manip;

    private NAR_Joystick leftStick;
    private NAR_Joystick rightStick;
    private NAR_ButtonBoard buttonPad;
    private NAR_XboxController operatorController;

    public static NAR_XboxController controller;

    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
  
    public static BooleanSupplier DEBUG = ()-> false; 

    private Trigger inProtected;

    public RobotContainer() {
        NAR_Shuffleboard.addData("DEBUG", "DEBUG", ()-> DEBUG.getAsBoolean(), 0, 1);
        var x = NAR_Shuffleboard.addData("DEBUG", "TOGGLE", false, 0, 0).withWidget("Toggle Button");
        DEBUG = ()-> x.getEntry().getBoolean(false);

        swerve = Swerve.getInstance();
        vision = Vision.getInstance();
        led = Led.getInstance();
        elevator = Elevator.getInstance();
        manip = ManipCRX.getInstance();

        //TODO: Enable all PIDSubsystems so that useOutput runs here
        // pivot.enable();
        // telescope.enable();

        leftStick = new NAR_Joystick(0);
        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_ButtonBoard(3);
        operatorController = new NAR_XboxController(4);

        // commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(rightStick::getX, rightStick::getY, rightStick::getZ, true));
        
        
        //uncomment line below to enable driving
        commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, true));
        initDashboard();
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        controller.getButton("A").onTrue(new InstantCommand(()-> Vision.AUTO_ENABLED = !Vision.AUTO_ENABLED));
        controller.getButton("RightTrigger").onTrue(new InstantCommand(()-> swerve.throttle = 1)).onFalse(new InstantCommand(()-> swerve.throttle = 0.8));
        controller.getButton("LeftTrigger").onTrue(new InstantCommand(()-> swerve.throttle = .25)).onFalse(new InstantCommand(()-> swerve.throttle = 0.8));
        controller.getButton("X").onTrue(new RunCommand(()-> swerve.xlock(), swerve)).onFalse(new InstantCommand(()-> swerve.stop(),swerve));
        controller.getButton("B").onTrue(new InstantCommand(()-> swerve.resetEncoders()));
        controller.getButton("Y").onTrue(runOnce(()-> ENABLE = true)).onFalse(runOnce(()-> ENABLE = false));
        controller.getButton("Start").onTrue(resetSwerve());
        controller.getButton("RightBumper").onTrue(pickup(Position.GROUND_CONE, true));
        controller.getButton("LeftBumper").onTrue(pickup(Position.GROUND_CUBE, true));
        
        rightStick.getButton(1).onTrue(resetSwerve());
        rightStick.getButton(2).onTrue(moveElv(0.4)).onFalse(moveElv(0));
        rightStick.getButton(3).onTrue(moveElv(-0.4)).onFalse(moveElv(0));
        rightStick.getButton(4).onTrue(moveElevator(30));
        rightStick.getButton(5).onTrue(resetElevator());
        rightStick.getButton(6).onTrue(moveWri(0.4)).onFalse(moveWri(0));
   
        rightStick.getButton(3).onTrue(new InstantCommand(()-> manip.intake(true))).onFalse(new InstantCommand(()->manip.stopRoller()));
        rightStick.getButton(4).onTrue(new InstantCommand(()-> manip.intake(false))).onFalse(new InstantCommand(()->manip.stopRoller()));
        rightStick.getButton(8).onTrue(new InstantCommand(()-> manip.stopRoller()));
        // rightStick.getButton(3).onTrue(CmdManipIntake(true));
        // rightStick.getButton(4).onTrue(CmdManipIntake(false));

        // rightStick.getButton(8).onTrue(CmdManipOutake()).onFalse(CmdManipStop());
        // rightStick.getButton(11).onTrue(CmdManipShoot()).onFalse(CmdManipStop());


        // rightStick.getButton(7).onTrue(moveWri(-0.4)).onFalse(moveWri(0));
        // rightStick.getButton(8).onTrue(moveWrist(30));
        // rightStick.getButton(9).onTrue(resetWrist());
        // rightStick.getButton(10).onTrue(intake(true));
        // rightStick.getButton(11).onTrue(intake(false));
        // rightStick.getButton(12).onTrue(outtake());
        // rightStick.getButton(13).onTrue(stopManip());


        buttonPad.getButton(5).onTrue(
            score(Position.LOW, 1)
        );
        buttonPad.getButton(8).onTrue(
            score(Position.MID_CUBE, 1)
        );
        buttonPad.getButton(11).onTrue(
            score(Position.HIGH_CUBE, 1)
        );

        buttonPad.getButton(13).onTrue(runOnce(()-> SINGLE_STATION = !SINGLE_STATION));

        buttonPad.getButton(14).onTrue(retract(Position.NEUTRAL));

        buttonPad.getButton(16).onTrue(
            HPpickup(Position.CHUTE_CONE, Position.SHELF_CONE)
        );

        buttonPad.getButton(15).onTrue(
            HPpickup(Position.CHUTE_CUBE, Position.SHELF_CUBE)
        );
        
        buttonPad.getButton(1).onTrue(runOnce (()-> {
            Vision.SELECTED_GRID = DriverStation.getAlliance() == Alliance.Red ? 0 : 2;
        }));
        buttonPad.getButton(2).onTrue(runOnce(()-> Vision.SELECTED_GRID = 1));
        buttonPad.getButton(3).onTrue(runOnce(()-> {
            Vision.SELECTED_GRID = DriverStation.getAlliance() == Alliance.Red ? 2 : 0;
        }));

        operatorController.getButton("Back").onTrue(new InstantCommand(()-> Vision.MANUAL = !Vision.MANUAL));

        // inProtected = new Trigger(
        //     () -> {
        //         Pose2d pose = Swerve.getInstance().getPose();
        //         if (DriverStation.getAlliance() == Alliance.Red) {
        //             return ((pose.getY() < midY + robotLength/2 && pose.getX() < outerX + robotLength/2) || 
        //                 (pose.getY() < leftY + robotLength/2 && pose.getX() < midX + robotLength/2)) ||
        //                 ((pose.getY() > 6.85 && pose.getX() > FIELD_X_LENGTH - 6.70) || (pose.getY() > 5.50 && pose.getX() > FIELD_X_LENGTH - 3.30));
        //         }
        //         return ((pose.getY() < midY + robotLength/2 && pose.getX() > FIELD_X_LENGTH - outerX - robotLength/2) || 
        //             (pose.getY() < leftY + robotLength/2 && pose.getX() > FIELD_X_LENGTH - midX - robotLength/2)) ||
        //             ((pose.getY() > 6.85 && pose.getX() < 6.70) || (pose.getY() > 5.50 && pose.getX() < 3.30));
        //     }
        // );
        // inProtected.onTrue(new InstantCommand(()-> controller.startVibrate())).onFalse(new InstantCommand(()-> controller.stopVibrate()));
    }

    public void init() {
        Vision.AUTO_ENABLED = false;
        if (DriverStation.getAlliance() == Alliance.Red) {
            buttonPad.getButton(4).onTrue(
                score(Position.LOW, 0)
            );
            buttonPad.getButton(6).onTrue(
                score(Position.LOW, 2)
            );
            buttonPad.getButton(7).onTrue(
                score(Position.MID_CONE, 0)
            );
            buttonPad.getButton(9).onTrue(
                score(Position.MID_CONE, 2)
            );
            buttonPad.getButton(10).onTrue(
                score(Position.HIGH_CONE, 0)
            );
            buttonPad.getButton(12).onTrue(
                score(Position.HIGH_CONE, 2)
            );
            
        }
        else {
            buttonPad.getButton(6).onTrue(
                score(Position.LOW, 0)
            );
            buttonPad.getButton(4).onTrue(
                score(Position.LOW, 2)
            );
            buttonPad.getButton(9).onTrue(
                score(Position.MID_CONE, 0)
            );
            buttonPad.getButton(7).onTrue(
                score(Position.MID_CONE, 2)
            );
            buttonPad.getButton(12).onTrue(
                score(Position.HIGH_CONE, 0)
            );
            buttonPad.getButton(10).onTrue(
                score(Position.HIGH_CONE, 2)
            );
        }
    }

    private void initDashboard() {
        if (DEBUG.getAsBoolean()) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            //SmartDashboard.putData("Swerve", swerve);
        }

        swerve.initShuffleboard();
        vision.initShuffleboard();

        NarwhalDashboard.startServer();
        
        Log.info("NarwhalRobot", "Setting up limelight chooser...");
      
        // for (NAR_Camera cam : vision.getCameras()) {
        //     NarwhalDashboard.addLimelight(cam);
        // }
    }

    public void updateDashboard() {
        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("x", swerve.getPose().getX());
        NarwhalDashboard.put("y", swerve.getPose().getY());
        SmartDashboard.putNumber("LeftX",controller.getLeftX());
        SmartDashboard.putNumber("LeftY",controller.getLeftY());
        SmartDashboard.putNumber("RightX",controller.getRightX());
        SmartDashboard.putNumber("RightY",controller.getRightY());
        NAR_Shuffleboard.update();
        SmartDashboard.putNumber("Pitch",swerve.getPitch());
    }
}
