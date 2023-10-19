package frc.team3128.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.team3128.PositionConstants.Position;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Swerve;
import static frc.team3128.commands.CmdManager.*;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    public static Swerve swerve;

    public AutoPrograms() {
        swerve = Swerve.getInstance();

        Trajectories.initTrajectories();
        initAutoSelector();
    }

    private void initAutoSelector() {
        String[] autoStrings = new String[] {
                                            //Blue Autos
                                                //Cable
                                                "b_cable_1Cone+1Cube","b_cable_1Cone+2Cube", //"b_cable_1Cone+2Cube+Climb",
                                                //Mid
                                                "b_mid_1Cone+Climb","b_mid_1Cone+1Cube+Climb",
                                                //Hp
                                                "b_hp_1Cone+1Cube",//"b_cable_1Cone+2Cube",
                                            
                                            //Red Autos
                                                //Cable
                                                "r_cable_1Cone+1Cube","r_cable_1Cone+2Cube",
                                                //Mid
                                                "r_mid_1Cone+Climb","r_mid_1Cone+1Cube+Climb",
                                                //Hp
                                                "r_hp_1Cone+1Cube"//"r_cable_1Cone+2Cube",

                                            };
        NarwhalDashboard.addAutos(autoStrings);
    }

    public Command getAutonomousCommand() {
       String selectedAutoName = NarwhalDashboard.getSelectedAutoName();
    //    String selectedAutoName = "b_cable_1Cone+1Cube";
        // String selectedAutoName = "b_cable_1Cone+2Cube+Climb"; //uncomment and change this for testing without opening Narwhal Dashboard
        // SmartDashboard.putString(selectedAutoName, selectedAutoName);
        if (selectedAutoName == null) {
            return score(Position.HIGH_CONE, true).beforeStarting(resetAuto());
        }

        return Trajectories.get(selectedAutoName,selectedAutoName.contains("Climb")).beforeStarting(resetAuto());
    }

    public CommandBase resetAuto() {
        return sequence(
            resetSwerve(DriverStation.getAlliance() == Alliance.Red ? 0 : 180),
            runOnce(()-> Manipulator.getInstance().set(-0.35), Manipulator.getInstance()),
            resetAll(),
            retract(Position.NEUTRAL)
        );
    }
    
    // /** 
    //  * Follow trajectory and intake balls along the path
    //  */
    // private SequentialCommandGroup IntakePathCmd(String trajectory) {
    //     ParallelDeadlineGroup movement = new ParallelDeadlineGroup(
    //                                         trajectoryCmd(trajectory), 
    //                                         new ScheduleCommand(new CmdExtendIntakeAndRun()));
    //     return new SequentialCommandGroup(new InstantCommand(intake::ejectIntake, intake), movement);
    // }

    /**
     * Flip 180 degrees rotation wise but keep same pose translation 
     */
    private Pose2d inverseRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), new Rotation2d(pose.getRotation().getRadians() + Math.PI));
    }
}