package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

public class Test {
    private static ShuffleboardTab testTab = Shuffleboard.getTab("Test");

    public static void addTestsToShuffleboard() {
        addCommandsToShuffleboard(

        );
    }

    private static void addCommandsToShuffleboard(Command... commands) {
        for (Command cmd : commands) {
            testTab.add(cmd);
        }
    }

}
