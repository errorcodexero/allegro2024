package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.SwerveDriveGamepad;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;

import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.intake_shooter.StartCollectAction;
import frc.robot.subsystems.intake_shooter.StopCollectAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Allegro2024OISubsystem extends OISubsystem {

    private final static SwerveDriveGamepad.SwerveButton[] resetButtons = { SwerveDriveGamepad.SwerveButton.Y, SwerveDriveGamepad.SwerveButton.B} ;
    private final static SwerveDriveGamepad.SwerveButton[] xActionButtons = { SwerveDriveGamepad.SwerveButton.LBack } ;

    private StartCollectAction start_collect_ ;
    private StopCollectAction stop_collect_ ;
    private IntakeShooterSubsystem intake_ ;

    public Allegro2024OISubsystem(Subsystem parent, DriveBaseSubsystem db) throws Exception {
        super (parent,"allegro2024oi",GamePadType.Swerve, db, true);

        AllegroRobot2024 robot = (AllegroRobot2024)parent.getRobot().getRobotSubsystem() ;
        IntakeShooterSubsystem sub = robot.getIntakeShooter() ;

        start_collect_ = new StartCollectAction(sub) ;
        stop_collect_ = new StopCollectAction(sub) ;
    }

    @Override
    protected void gamePadCreated(Gamepad gp) {
        SDSSwerveDriveSubsystem swerve = (SDSSwerveDriveSubsystem)getRobot().getRobotSubsystem().getDB();

        SwerveDriveGamepad swgp = (SwerveDriveGamepad)gp ;
        if (swgp != null) {
            swgp.bindButtons(resetButtons, 
                                ()->swgp.resetSwerveDriveDirection(), 
                                null) ;
            swgp.bindButtons(xActionButtons, 
                                ()->swgp.startDriveBaseX(), 
                                ()->swgp.stopDriveBaseX()) ;
            swgp.bindButton(SwerveDriveGamepad.SwerveButton.RTrigger, 
                                ()-> { intake_.setAction(start_collect_) ; }, 
                                ()-> { intake_.setAction(stop_collect_) ; }) ;
            swgp.bindButton(SwerveDriveGamepad.SwerveButton.LTrigger,
                                () -> { swerve.setRotationSWControl(true);},
                                () -> { swerve.setRotationSWControl(false);}) ;
        }
    }    
}
