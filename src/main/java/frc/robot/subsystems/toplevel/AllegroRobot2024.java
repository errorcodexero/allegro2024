package frc.robot.subsystems.toplevel;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import frc.robot.subsystems.amp_trap.AmpTrapSubsystem;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.lidar.LidarSubsystem;
import frc.robot.subsystems.oi.Allegro2024OISubsystem;

public class AllegroRobot2024 extends RobotSubsystem {

    private SDSSwerveDriveSubsystem db_;
    private Allegro2024OISubsystem oi_;
    private LimeLightSubsystem ll_;
    private IntakeShooterSubsystem is_;
    private AmpTrapSubsystem at_;
    private LidarSubsystem lidar_;

    public AllegroRobot2024(XeroRobot robot) throws Exception {
        super(robot, "Allegro2024RobotSubsystem");

        is_ = new IntakeShooterSubsystem(this);
        addChild(is_);        

        db_ = new SDSSwerveDriveSubsystem(this, "swerve");
        addChild(db_);

        oi_ = new Allegro2024OISubsystem(this, db_, is_);
        addChild(oi_);

        ll_ = new LimeLightSubsystem(this, "limelight");
        addChild(ll_);

        if (XeroRobot.isSimulation()) {
            at_ = new AmpTrapSubsystem(this);
            addChild(at_);

            lidar_ = new LidarSubsystem(this);
            addChild(lidar_);
        }
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
        return is_;
    }

    public AmpTrapSubsystem getAmpTrap() {
        return at_;
    }

    public LidarSubsystem getLidar() {
        return lidar_;
    }
}
