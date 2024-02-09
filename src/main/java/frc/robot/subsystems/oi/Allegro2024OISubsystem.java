package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.SwerveDriveGamepad;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;

import frc.robot.subsystems.intake_shooter.ButchStartCollectAction;
import frc.robot.subsystems.intake_shooter.ButchStopCollectionAction;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.intake_shooter.ManualShootAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Allegro2024OISubsystem extends OISubsystem {

    private final static SwerveDriveGamepad.SwerveButton[] resetButtons = { SwerveDriveGamepad.SwerveButton.Y, SwerveDriveGamepad.SwerveButton.B} ;

    private ButchStartCollectAction startCollectAction_ ;
    private ButchStopCollectionAction stopCollectAction_ ;

    public Allegro2024OISubsystem(Subsystem parent, DriveBaseSubsystem db, IntakeShooterSubsystem intake) throws Exception {
        super (parent,"allegro2024oi",GamePadType.Swerve, db, true);

        startCollectAction_ = new ButchStartCollectAction(intake);
        stopCollectAction_ = new ButchStopCollectionAction(intake);
    }

    public void startCollect() {
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
        IntakeShooterSubsystem intake = robot.getIntakeShooter();

        intake.setAction(startCollectAction_) ;
    }

    public void stopCollect() {
        AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
        IntakeShooterSubsystem intake = robot.getIntakeShooter();

        intake.setAction(stopCollectAction_) ;
    }

    public void startTargetLockMode() {
        SDSSwerveDriveSubsystem sdb = (SDSSwerveDriveSubsystem)getRobot().getRobotSubsystem().getDB() ;
        sdb.setRotationSWControl(true);
    }

    public void stopTargetLockMode() {
        SDSSwerveDriveSubsystem sdb = (SDSSwerveDriveSubsystem)getRobot().getRobotSubsystem().getDB() ;
        sdb.setRotationSWControl(false);
    }

    public void manualShoot() {
        try {
            AllegroRobot2024 robot = (AllegroRobot2024)getRobot().getRobotSubsystem();
            IntakeShooterSubsystem intake = robot.getIntakeShooter();
            intake.setAction(new ManualShootAction(intake));    
        }
        catch(Exception ex) {
        }
    }

    @Override
    protected void gamePadCreated(Gamepad gp) {
        SwerveDriveGamepad swgp = (SwerveDriveGamepad)gp ;
        if (swgp != null) {
            swgp.bindButtons(resetButtons, ()->swgp.resetSwerveDriveDirection(), null);            
            swgp.bindButton(SwerveDriveGamepad.SwerveButton.LBack, ()-> swgp.startDriveBaseX(), ()->swgp.stopDriveBaseX());   
            swgp.bindButton(SwerveDriveGamepad.SwerveButton.RBack, ()->startCollect(), ()->stopCollect());
            swgp.bindButton(SwerveDriveGamepad.SwerveButton.LTrigger, ()->startTargetLockMode(), null);
            swgp.bindButton(SwerveDriveGamepad.SwerveButton.RTrigger, ()->stopTargetLockMode(), null);
            swgp.bindButton(SwerveDriveGamepad.SwerveButton.A, ()->manualShoot(), null);
        }
    }    
}