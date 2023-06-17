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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.IntakeConstants;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.Constants.VisionConstants;
import frc.team3128.Constants.ArmConstants.ArmPosition;
import frc.team3128.commands.CmdBangBangBalance;
import frc.team3128.commands.CmdDriveUp;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.commands.CmdBalance;
import frc.team3128.commands.CmdMove;
import frc.team3128.commands.CmdMovePickup;
import frc.team3128.commands.CmdMoveScore;
import frc.team3128.commands.CmdMove.Type;
import frc.team3128.subsystems.Led;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;
import static frc.team3128.commands.CmdManager.*;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static HashMap<String, List<PathPlannerTrajectory>> trajectories = new HashMap<String, List<PathPlannerTrajectory>>();

    private static SwerveAutoBuilder builder;

    private static HashMap<String, Command> CommandEventMap = new HashMap<String, Command>();

    private static Swerve swerve = Swerve.getInstance();

    public static double autoSpeed = SwerveConstants.maxSpeed;

    // private static Intake intake = Intake.getInstance();

    public static void initTrajectories() {
        final String[] trajectoryNames = {"r_top_1Cone", "r_top_1Cone+1Cube", "r_top_1Cone+1Cube+Climb",
                                            "b_top_1Cone", "b_top_1Cone+1Cube", "b_top_1Cone+1Cube+Climb",

                                            "r_mid_1Cone", "r_mid_1Cone+Climb",
                                            "b_mid_1Cone", "b_mid_1Cone+Climb",

                                            "r_bottom_1Cone", "r_bottom_1Cone+1Cube", "r_bottom_1Cone+1Cube+Climb",
                                            "b_bottom_1Cone", "b_bottom_1Cone+1Cube", "b_bottom_1Cone+1Cube+Climb"
                                            };
        
        CommandEventMap.put("ClimbPoseBlue", new CmdMove(Type.SCORE, false, new Pose2d(5.8,2.7,Rotation2d.fromDegrees(0))));
        
        CommandEventMap.put("ClimbPoseRed", new CmdMove(Type.SCORE, false, new Pose2d(10.7,2.7,Rotation2d.fromDegrees(0))));
        
        for (String trajectoryName : trajectoryNames) {
            // Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, new PathConstraints(maxSpeed, maxAcceleration)));
        }

        builder = new SwerveAutoBuilder(
            Swerve.getInstance()::getPose,
            Swerve.getInstance()::resetOdometry,
            swerveKinematics,
            new PIDConstants(1,0,0),
            new PIDConstants(1,0,0),
            Swerve.getInstance()::setModuleStates,
            CommandEventMap,
            Swerve.getInstance()
        );
    }

    public static CommandBase get(String name) {
        return builder.fullAuto(trajectories.get(name));
    }

    public static PathPlannerTrajectory line(Pose2d start, Pose2d end) {
        return PathPlanner.generatePath(
            new PathConstraints(maxSpeed, maxAcceleration), 
            new PathPoint(start.getTranslation(), start.getRotation()), 
            new PathPoint(end.getTranslation(), end.getRotation())
            );
    }

    public static CommandBase lineCmd(Pose2d start, Pose2d end) {
        return builder.fullAuto(line(start, end));
    }

    public static CommandBase movePoint(Pose2d pose) {
        return Commands.sequence(
            new InstantCommand(()->Vision.AUTO_ENABLED = true),
            new CmdMovePickup(false, autoSpeed, pose)
        ); 
    }
    
}