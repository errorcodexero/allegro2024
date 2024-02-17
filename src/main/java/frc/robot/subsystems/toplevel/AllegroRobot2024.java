package frc.robot.subsystems.toplevel;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import frc.robot.subsystems.amp_trap.AmpTrapSubsystem;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.lidar.LidarSubsystem;
import frc.robot.subsystems.oi.Allegro2024OISubsystem;
import frc.robot.subsystems.superstructure.SuperStructureSubsystem;
import frc.robot.subsystems.target_tracker.TargetTrackerSubsystem;

public class AllegroRobot2024 extends RobotSubsystem {

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

        tt_ = new TargetTrackerSubsystem(this);
        addChild(tt_);

        lidar_ = new LidarSubsystem(this);
        addChild(lidar_) ;

        db_.setVision(ll_);
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
}
