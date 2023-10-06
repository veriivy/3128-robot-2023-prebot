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
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.PositionConstants.Position;
import static frc.team3128.commands.CmdManager.*;

import frc.team3128.commands.CmdAutoBalance;
import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Swerve;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static HashMap<String, List<PathPlannerTrajectory>> trajectories = new HashMap<String, List<PathPlannerTrajectory>>();

    private static SwerveAutoBuilder builder;

    private static HashMap<String, Command> CommandEventMap = new HashMap<String, Command>();

    private static Manipulator manipulator = Manipulator.getInstance();

    private static Swerve swerve = Swerve.getInstance();

    public static double autoSpeed = SwerveConstants.maxSpeed;


    public static void initTrajectories() {
        final String[] trajectoryNames = {
                                        //Blue Autos
                                            //Cable
                                            "b_cable_1Cone+1Cube","b_cable_1Cone+2Cube", "b_cable_1Cone+2Cube+Climb",
                                            //Mid
                                            "b_mid_1Cone+Climb","b_mid_1Cone+1Cube+Climb",
                                            //Hp
                                            "b_hp_1Cone+1Cube","b_cable_1Cone+2Cube",
                                            
                                        //Red Autos
                                            //Cable
                                            "r_cable_1Cone+1Cube","r_cable_1Cone+2Cube",
                                            //Mid
                                            "r_mid_1Cone+Climb","r_mid_1Cone+1Cube+Climb",
                                            //Hp
                                            "r_hp_1Cone+1Cube","r_cable_1Cone+2Cube",
                                        };

        CommandEventMap.put("ScoreConeHigh", score(Position.HIGH_CONE, true));

        CommandEventMap.put("ScoreCubeHigh", score(Position.HIGH_CUBE, true));

        CommandEventMap.put("ScoreLow", score(Position.LOW, true));
        
        CommandEventMap.put("PickupCube", pickup(Position.GROUND_CUBE, true));

        CommandEventMap.put("Neutral", retract(Position.NEUTRAL));

        for (String trajectoryName : trajectoryNames) {
            // Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            if (trajectoryName.contains("mid")) {
                trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, new PathConstraints(AutoConstants.slowSpeed, AutoConstants.slowAcceleration)));
            } 
            else {
                trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, new PathConstraints(maxSpeed, maxAcceleration)));
            }
        }

        builder = new SwerveAutoBuilder(
            Swerve.getInstance()::getPose,
            Swerve.getInstance()::resetOdometry,
            swerveKinematics,
            new PIDConstants(translationKP,translationKI,translationKD),
            new PIDConstants(rotationKP,rotationKI,rotationKD),
            Swerve.getInstance()::setModuleStates,
            CommandEventMap,
            Swerve.getInstance()
        );
    }

    public static CommandBase generateAuto(PathPlannerTrajectory trajectory) {
        return builder.fullAuto(trajectory);
    }

    public static CommandBase get(String name, boolean balance) {
        if (balance) {
            return sequence(
                builder.fullAuto(trajectories.get(name)),
                new CmdAutoBalance(true)
            );
        }
        return builder.fullAuto(trajectories.get(name));
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