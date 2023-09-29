package frc.team3128.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.SwerveConstants;
import frc.team3128.subsystems.Swerve;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static HashMap<String, List<PathPlannerTrajectory>> trajectories = new HashMap<String, List<PathPlannerTrajectory>>();

    public static SwerveAutoBuilder builder;

    private static HashMap<String, Command> CommandEventMap = new HashMap<String, Command>();

    private static Swerve swerve = Swerve.getInstance();

    public static double autoSpeed = SwerveConstants.maxSpeed;


    public static void initTrajectories() {
        final String[] trajectoryNames = {"TestAuto1", "b_bottom_1Cone+1Cube","b_bottom_1Cone"};

        CommandEventMap.put("ScoreConeHigh", Commands.sequence());
        CommandEventMap.put("ScoreCubeLow", Commands.sequence());
        
        CommandEventMap.put("IntakeCube", Commands.sequence());

        CommandEventMap.put("RetractIntake", Commands.sequence());


        for (String trajectoryName : trajectoryNames) {
            // Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, new PathConstraints(maxSpeed, maxAcceleration)));
        }

        builder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            swerveKinematics,
            new PIDConstants(translationKP,translationKI,translationKD),
            new PIDConstants(rotationKP,rotationKI,rotationKD),
            swerve::setModuleStates,
            CommandEventMap,
            swerve
        );
    }

    public static CommandBase get(String name) {
        return builder.fullAuto(trajectories.get(name));
    }

    public static CommandBase generateAuto(PathPlannerTrajectory trajectory) {
        return builder.fullAuto(trajectory);
    }

    public static PathPlannerTrajectory line(Pose2d start, Pose2d end) {
        return PathPlanner.generatePath(
            new PathConstraints(maxSpeed, 4), 
            new PathPoint(start.getTranslation(), start.getRotation()), 
            new PathPoint(end.getTranslation(), end.getRotation())
            );
    }

    public static CommandBase lineCmd(Pose2d start, Pose2d end) {
        return builder.fullAuto(line(start, end));
    }
    
}