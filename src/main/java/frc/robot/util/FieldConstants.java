package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import frc.lib.FieldFlipper;


public class FieldConstants {
     public static final Pose2d BLUE_HUB_POSE = 
            new Pose2d(1,1,Rotation2d.kZero);

        public static final Pose2d RED_HUB_POSE = 
            FieldFlipper.flip(BLUE_HUB_POSE);

       

}
