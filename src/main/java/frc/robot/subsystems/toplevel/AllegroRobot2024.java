package frc.robot.subsystems.toplevel;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import frc.robot.subsystems.intakeshooter.IntakeShooterSubsystem;
import frc.robot.subsystems.oi.Allegro2024OISubsystem;
import frc.robot.subsystems.protointake.ProtoTypeIntakeSubsystem;

public class AllegroRobot2024 extends RobotSubsystem {

    private final static boolean kUsePrototypeIntake = false ;

    private SDSSwerveDriveSubsystem db_;
    private Allegro2024OISubsystem oi_;
    private LimeLightSubsystem ll_;
    private IntakeShooterSubsystem intake_ ;
    private ProtoTypeIntakeSubsystem proto_intake_ ;

    public AllegroRobot2024(XeroRobot robot) throws Exception {
        super(robot, "Allegro2024RobotSubsystem");

        db_ = new SDSSwerveDriveSubsystem(this, "swerve");
        addChild(db_);

        oi_ = new Allegro2024OISubsystem(this, db_);
        addChild(oi_);

        ll_ = new LimeLightSubsystem(this, "limelight");
        addChild(ll_);

        if (kUsePrototypeIntake) {
            proto_intake_ = new ProtoTypeIntakeSubsystem(this) ;
            addChild(proto_intake_) ;
        }
        else {
            intake_ = new IntakeShooterSubsystem(this);
            addChild(intake_);
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

    public IntakeShooterSubsystem getIntakeShooterSubsystem() {
        return intake_ ;
    }
}
