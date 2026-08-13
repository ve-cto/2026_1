package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// Modified from team 3414's Auto Builder https://github.com/hackbots-3414/2026_Rebuilt/
public class BetterAutoChooser {
  private static final boolean RIGHT_IS_FLIPPED = true;
  private static final String DEFAULT_NAME = "None";
  private static final String FLIP_PREFIX = "FLIP ";

  private static Map<String, Pose2d> startingPoses = new HashMap<>();

  private static SendableChooser<Command> chooser;

  public static SendableChooser<Command> buildAutoChooser() {
    chooser = new SendableChooser<>();
    SmartDashboard.putData("Auto Chooser", chooser);
    chooser.setDefaultOption(DEFAULT_NAME, Commands.none());
    if (!AutoBuilder.isConfigured()) {
      return chooser;
    }

    for (String name : AutoBuilder.getAllAutoNames()) {
      if (name.startsWith(FLIP_PREFIX)) {
        // Make the flipped paths
        String remainder = name.substring(FLIP_PREFIX.length());
        registerAuto("Right " + remainder + " (Generated)", new PathPlannerAuto(name, RIGHT_IS_FLIPPED));
        registerAuto("Left " + remainder + " (Generated)", new PathPlannerAuto(name, !RIGHT_IS_FLIPPED));
      } else {
        registerAuto(name, new PathPlannerAuto(name));
      }
    }

    return chooser;
  }

  private static void registerAuto(String name, PathPlannerAuto auto) {
    chooser.addOption(name, auto.withName(name));
    Pose2d startingPose = auto.getStartingPose();
    if (startingPose != null) {
      startingPoses.put(name, startingPose);
    }
  }
}
