package frc.robot.subsystems.toplevel;

import org.xero1425.base.LoopType;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem.LedMode;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.AprilTags;
import frc.robot.subsystems.amp_trap.AmpTrapSubsystem;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.lidar.LidarSubsystem;
import frc.robot.subsystems.oi.Allegro2024OISubsystem;
import frc.robot.subsystems.superstructure.SuperStructureSubsystem;
import frc.robot.subsystems.target_tracker.TargetTrackerSubsystem;

public class AllegroRobot2024 extends RobotSubsystem {

    public class StageTag {
        public final int id_ ;
        public final Pose2d pose_ ;
        public final double distance_ ;

        public StageTag(int id, Pose2d p, double dist) {
            id_ = id ;
            pose_ = p ;
            distance_ = dist ;
        }
    }

    private SDSSwerveDriveSubsystem db_;
    private Allegro2024OISubsystem oi_;
    private LimeLightSubsystem ll_;
    private LidarSubsystem lidar_;
    private TargetTrackerSubsystem tt_ ;
    private SuperStructureSubsystem ss_ ;

    public AllegroRobot2024(XeroRobot robot) throws Exception {
        super(robot, "Allegro2024RobotSubsystem");
        
        ss_ = new SuperStructureSubsystem(this);
        addChild(ss_) ;

        db_ = new SDSSwerveDriveSubsystem(this, "swerve");
        addChild(db_);

        oi_ = new Allegro2024OISubsystem(this, db_, ss_.getIntakeShooter()) ;
        addChild(oi_);

        ll_ = new LimeLightSubsystem(this, "limelight");
        addChild(ll_);

        tt_ = new TargetTrackerSubsystem(this, ll_);
        addChild(tt_);

        lidar_ = new LidarSubsystem(this);
        addChild(lidar_) ;

        db_.setVision(ll_);

        ll_.setLedMode(LedMode.ForceOff);
    }

    @Override
    public  void init(LoopType prev, LoopType next) {
        super.init(prev, next) ;
    }

    public SuperStructureSubsystem getSuperStructure() {
        return ss_ ;
    }

    public SDSSwerveDriveSubsystem getSwerve() {
        return db_;
    }

    public Allegro2024OISubsystem getOI() {
        return oi_;
    }

    public LimeLightSubsystem getLimelight() {
        return ll_;
    }

    public IntakeShooterSubsystem getIntakeShooter() {
        return ss_.getIntakeShooter();
    }

    public AmpTrapSubsystem getAmpTrap() {
        return ss_.getAmpTrap();
    }

    public LidarSubsystem getLidar() {
        return lidar_;
    }

    public TargetTrackerSubsystem getTargetTracker() {
        return tt_ ;
    }

    public StageTag getStagePose() {
        StageTag tag = null ;
        Pose2d pose = null ;
        double dist = 0.0 ;
        int id = -1 ;

        var alliance = DriverStation.getAlliance() ;
        if (alliance.isPresent()) {
            AprilTagFieldLayout tags = getRobot().getAprilTags() ;
            if (alliance.get() == DriverStation.Alliance.Blue) {
                if (ll_.hasAprilTag(AprilTags.BLUE_STAGE_LEFT)) {
                    id = AprilTags.BLUE_STAGE_LEFT ;
                    pose = tags.getTagPose(AprilTags.BLUE_STAGE_LEFT).get().toPose2d() ;
                }
                else if (ll_.hasAprilTag(AprilTags.BLUE_STAGE_RIGHT)) {
                    id = AprilTags.BLUE_STAGE_RIGHT ;
                    pose = tags.getTagPose(AprilTags.BLUE_STAGE_RIGHT).get().toPose2d() ; ;
                }
                else if (ll_.hasAprilTag(AprilTags.BLUE_CENTER_STAGE)) {
                    id = AprilTags.BLUE_CENTER_STAGE ;
                    pose = tags.getTagPose(AprilTags.BLUE_CENTER_STAGE).get().toPose2d() ; ;
                }
            }
            else {
                if (ll_.hasAprilTag(AprilTags.RED_STAGE_LEFT)) {
                    id = AprilTags.RED_STAGE_LEFT ;
                    pose = tags.getTagPose(AprilTags.RED_STAGE_LEFT).get().toPose2d() ;
                }
                else if (ll_.hasAprilTag(AprilTags.RED_STAGE_RIGHT)) {
                    id = AprilTags.RED_STAGE_RIGHT ;
                    pose = tags.getTagPose(AprilTags.RED_STAGE_RIGHT).get().toPose2d() ; ;
                }
                else if (ll_.hasAprilTag(AprilTags.RED_CENTER_STAGE)) {
                    id = AprilTags.RED_CENTER_STAGE ;
                    pose = tags.getTagPose(AprilTags.RED_CENTER_STAGE).get().toPose2d() ; ;
                }
            }
        }

        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;     

        if (pose != null) {
            dist = db_.getPose().getTranslation().getDistance(pose.getTranslation()) ;
            tag = new StageTag(id, pose, dist);

            logger.add("stage tag found ") ;
            logger.add("id", id) ;
            logger.add("distance", dist) ;
            logger.add("tagpose", pose.toString());
            logger.add("dbpose", db_.getPose().toString()) ;
        }
        else {
            logger.add("no stage tag found") ;
        }
        logger.endMessage();

        return tag ;
    }
}
