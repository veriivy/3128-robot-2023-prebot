package frc.team3128.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.team3128.PositionConstants.Position;
import frc.team3128.commands.CmdAutoBalance;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import static frc.team3128.commands.CmdManager.*;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    public AutoPrograms() {

        Trajectories.initTrajectories();
        initAutoSelector();
    }

    private void initAutoSelector() {
        String[] autoStrings = new String[] {
                                            //Blue Autos
                                                //Cable
                                                "cable_1Cone+1Cube","cable_1Cone+2Cube",
                                                //Mid
                                                "mid_1Cone+Climb","mid_1Cone+1Cube+Climb",
                                                //Hp
                                                "hp_1Cone+1Cube",

                                                "scuffedClimb"
                                            };
        NarwhalDashboard.addAutos(autoStrings);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getSelectedAutoName();

        if (selectedAutoName == null) {
            return score(Position.HIGH_CONE, true).beforeStarting(Trajectories.resetAuto());
        }

        if (selectedAutoName == "scuffedClimb") {
            return sequence(
                score(Position.HIGH_CONE, true).beforeStarting(Trajectories.resetAuto()),
                new CmdAutoBalance(false)
            );
        }

        selectedAutoName = (DriverStation.getAlliance() == Alliance.Red) ? "r_" : "b_" + selectedAutoName;

        return Trajectories.get(selectedAutoName);
    }
}