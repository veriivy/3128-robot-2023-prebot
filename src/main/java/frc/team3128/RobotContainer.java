package frc.team3128;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.PositionConstants.Position;
import static frc.team3128.commands.CmdManager.*;
import frc.team3128.common.hardware.input.NAR_ButtonBoard;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Elevator;
import frc.team3128.subsystems.Leds;
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
    private Leds leds;
    private Elevator elevator;

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
        leds = Leds.getInstance();
        elevator = Elevator.getInstance();

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
        controller.getButton("RightTrigger").onTrue(score(Position.LOW, true)).onFalse(runOnce(()-> ENABLE = false));
        controller.getButton("LeftTrigger").onTrue(runOnce(()-> ENABLE = true)).onFalse(runOnce(()-> ENABLE = false));
        controller.getButton("X").onTrue(new RunCommand(()-> swerve.xlock(), swerve)).onFalse(new InstantCommand(()-> swerve.stop(),swerve));
        controller.getButton("B").onTrue(new InstantCommand(()-> swerve.resetEncoders()));
        controller.getButton("Y").onTrue(runOnce(()-> swerve.throttle = 1)).onFalse(runOnce(()-> swerve.throttle = 0.8));
        controller.getButton("Start").onTrue(resetSwerve());
        controller.getButton("RightBumper").onTrue(pickup(Position.GROUND_CUBE, true));
        controller.getButton("LeftBumper").onTrue(pickup(Position.GROUND_CONE, true));

        controller.getUpPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.rSetpoint = DriverStation.getAlliance() == Alliance.Red ? 0 : 180;
            CmdSwerveDrive.enabled = true;
        }));
        controller.getDownPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.rSetpoint = DriverStation.getAlliance() == Alliance.Red ? 180 : 0;
            CmdSwerveDrive.enabled = true;
        }));

        controller.getRightPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.rSetpoint = DriverStation.getAlliance() == Alliance.Red ? 270 : 90;
            CmdSwerveDrive.enabled = true;
        }));

        controller.getLeftPOVButton().onTrue(runOnce(()-> {
            CmdSwerveDrive.rSetpoint = DriverStation.getAlliance() == Alliance.Red ? 90 : 270;
            CmdSwerveDrive.enabled = true;
        }));
        
        rightStick.getButton(1).onTrue(resetSwerve());
        rightStick.getButton(2).onTrue(moveElv(0.4)).onFalse(moveElv(0));
        rightStick.getButton(3).onTrue(moveElv(-0.4)).onFalse(moveElv(0));
        rightStick.getButton(4).onTrue(moveElevator(30));
        rightStick.getButton(5).onTrue(resetElevator());
        rightStick.getButton(6).onTrue(moveWri(0.4)).onFalse(moveWri(0));
   
   
        rightStick.getButton(7).onTrue(moveWri(-0.4)).onFalse(moveWri(0));
        rightStick.getButton(8).onTrue(moveWrist(30));
        rightStick.getButton(9).onTrue(resetWrist());
        rightStick.getButton(10).onTrue(intake(true));
        rightStick.getButton(11).onTrue(intake(false));
        rightStick.getButton(12).onTrue(outtake());
        rightStick.getButton(13).onTrue(stopManip());


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

    }

    public void init() {
        leds.setElevatorLeds(Colors.DEFAULT);
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
            // SmartDashboard.putData("Swerve", swerve);
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
