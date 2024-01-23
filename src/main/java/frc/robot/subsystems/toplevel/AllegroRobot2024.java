package frc.robot.subsystems.toplevel;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.oi.Allegro2024OISubsystem;

public class AllegroRobot2024 extends RobotSubsystem {

    private SDSSwerveDriveSubsystem db_;
    private Allegro2024OISubsystem oi_;
    private LimeLightSubsystem ll_;
    private IntakeShooterSubsystem intake_shooter_ ;

    public AllegroRobot2024(XeroRobot robot) throws Exception {
        super(robot, "Allegro2024RobotSubsystem");

        db_ = new SDSSwerveDriveSubsystem(this, "swerve");
        addChild(db_);

        oi_ = new Allegro2024OISubsystem(this, db_);
        addChild(oi_);

        ll_ = new LimeLightSubsystem(this, "limelight");
        addChild(ll_);

        intake_shooter_ = new IntakeShooterSubsystem(this);
        addChild(intake_shooter_);
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

    public IntakeShooterSubsystem getIntakeShooterSubsystem() {
        return intake_shooter_ ;
    }
}
