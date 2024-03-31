package org.xero1425.base.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Pose2dWithRotation extends Pose2d {
    private Rotation2d rotation_ ;
    
    public Pose2dWithRotation(double x, double y, Rotation2d heading, Rotation2d rotation) {
        super(new Translation2d(x, y), heading) ;
        rotation_ = rotation ;
    }

    public Pose2dWithRotation(Pose2d pose, Rotation2d rotation) {
        super(pose.getTranslation(), pose.getRotation()) ;
        rotation_ = rotation ;
    }    

    public Rotation2d getRobotRotation() {
        return rotation_ ;
    }
}
