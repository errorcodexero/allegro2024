package frc.robot.subsystems.toplevel;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.ampTrap.AmpTrapSubsystem;
import frc.robot.subsystems.oi.Allegro2024OISubsystem;
import frc.robot.subsystems.targettracker.TargetTrackerSubsystem;

public class AllegroRobot2024 extends RobotSubsystem {

    private SDSSwerveDriveSubsystem db_;
    private Allegro2024OISubsystem oi_;
    private LimeLightSubsystem ll_;
    private IntakeShooterSubsystem is_;
    private AmpTrapSubsystem at_;
    private TargetTrackerSubsystem tt_ ;

    public AllegroRobot2024(XeroRobot robot) throws Exception {
        super(robot, "Allegro2024RobotSubsystem");

        db_ = new SDSSwerveDriveSubsystem(this, "swerve");
        addChild(db_);

        oi_ = new Allegro2024OISubsystem(this, db_);
        addChild(oi_);

        ll_ = new LimeLightSubsystem(this, "limelight");
        addChild(ll_);

        is_ = new IntakeShooterSubsystem(db_);
        addChild(is_);
      
        at_ = new AmpTrapSubsystem(this);
        addChild(at_);

        tt_ = new TargetTrackerSubsystem(this, db_, ll_);
        addChild(tt_);

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

    public AmpTrapSubsystem getAmpTrap(){
        return at_;
    }  
}
