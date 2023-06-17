package frc.team3128.autonomous;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam, Leo Lesmes
 */

public class AutoPrograms {
    private HashMap<String, Command> auto;
    public Swerve swerve;
    public Vision vision;

    public AutoPrograms() {
        swerve = Swerve.getInstance();
        vision = Vision.getInstance();

        // Trajectories.initTrajectories();
        initAutoSelector();
    }

    private void initAutoSelector() {

        auto = new HashMap<String, Command>();
        
        var array = auto.keySet();

        var arrayCopy = new String[array.size()];
        int index = 0;
        for (String x : array) {
            arrayCopy[index] = x;
            index++;
        }

        NarwhalDashboard.addAutos(arrayCopy);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = "bottom_1pc+mobility";//NarwhalDashboard.getSelectedAutoName();
        // String selectedAutoName = "bottom_2.5pc+Climb"; //uncomment and change this for testing without opening Narwhal Dashboard
        //REMINDER TO TEST AUTO SPEED AT SOME POINT
        if (selectedAutoName == null) {
            return auto.get("DEFAULT");
        }

        return auto.get(selectedAutoName);
    }
}