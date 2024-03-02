package frc.robot.automodes;

import org.xero1425.base.actions.ParallelAction;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.actions.ParallelAction.DonePolicy;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.subsystems.swerve.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.SwerveTrackAngle;
import org.xero1425.misc.ISettingsSupplier;

import frc.robot.subsystems.intake_shooter.StartCollectAltAction;
import frc.robot.subsystems.target_tracker.TargetTrackerSubsystem;
import frc.robot.subsystems.intake_shooter.IntakeAutoShootAction;
import frc.robot.subsystems.intake_shooter.IntakeShooterSubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

//
// For now these are set up assuming the path following is sufficient
// to shoot the note.  If this is not true
//

public class AllegroGameAutoMode extends AutoMode {
    private final static boolean kRequireAprilTagAndRotate = false ;
    private final static double kCollectEndOfPathDistance = 0.5 ;
    private final static double kShootEndOfPathDistance = 0.08 ;

    private StartCollectAltAction start_collect_ ;
    private boolean mirror_ ;
    private double mvalue_ ;

    public AllegroGameAutoMode(AutoController ctrl, String name, boolean mirror, double mvalue) {
        super(ctrl, name) ;

        mirror_ = mirror ;
        mvalue_ = mvalue ;
    }

    protected void shootFirstNote() throws Exception {
        AllegroRobot2024 robot = (AllegroRobot2024)getAutoController().getRobot().getRobotSubsystem() ;
        SwerveBaseSubsystem swerve = robot.getSwerve() ;
        TargetTrackerSubsystem tracker = robot.getTargetTracker() ;
        SwerveTrackAngle rotate = null ;
 
        if (kRequireAprilTagAndRotate) {
            ISettingsSupplier settings = robot.getRobot().getSettingsSupplier() ;
            IntakeShooterSubsystem intake = robot.getIntakeShooter() ;
            double postol = intake.getSettingsValue("actions:auto-shoot:rotational-position-tolerance").getDouble() ;
            rotate = new SwerveTrackAngle(swerve, () -> tracker.getRotation(), postol) ;
            addSubActionPair(robot.getSwerve(), rotate, false);
        }

        IntakeAutoShootAction shoot = new IntakeAutoShootAction(robot.getIntakeShooter(), robot.getTargetTracker(), true, rotate) ;
        addSubActionPair(robot.getIntakeShooter(), shoot, true);
    }

    protected void driveAndCollect(String path, boolean setpose) throws Exception {
        AllegroRobot2024 robot = (AllegroRobot2024)getAutoController().getRobot().getRobotSubsystem() ;        
        SwerveHolonomicPathFollower pathact = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 0.2, mirror_, mvalue_);
        start_collect_ = new StartCollectAltAction(robot.getIntakeShooter()) ;

        double collectLength = pathact.getDistance() - kCollectEndOfPathDistance ;

        pathact.addDistanceBasedAction(collectLength, () -> { robot.getIntakeShooter().setAction(start_collect_); });
        addSubActionPair(robot.getSwerve(), pathact, true) ;
    }

    protected void driveAndShoot(String path, boolean setpose) throws Exception {
        AllegroRobot2024 robot = (AllegroRobot2024)getAutoController().getRobot().getRobotSubsystem() ;        
        SwerveHolonomicPathFollower pathact = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 0.2, mirror_, mvalue_);
        SwerveTrackAngle rotate = null ;

        if (kRequireAprilTagAndRotate) {
            ISettingsSupplier settings = robot.getRobot().getSettingsSupplier() ;
            IntakeShooterSubsystem intake = robot.getIntakeShooter() ;
            double postol = intake.getSettingsValue("actions:auto-shoot:rotational-position-tolerance").getDouble() ;
            rotate = new SwerveTrackAngle(robot.getSwerve(), () -> robot.getTargetTracker().getRotation(), postol) ;
            addSubActionPair(robot.getSwerve(), rotate, false);
        }

        IntakeAutoShootAction shoot = new IntakeAutoShootAction(robot.getIntakeShooter(), robot.getTargetTracker(), true, rotate);

        double shootdist = pathact.getDistance() - kShootEndOfPathDistance ;
        pathact.addDistanceBasedAction(shootdist, ()-> { shoot.setDriveTeamReady(true);}) ;

        ParallelAction action = new ParallelAction(getAutoController().getRobot().getMessageLogger(), DonePolicy.All) ;
        action.addSubActionPair(robot.getSwerve(), pathact, true) ;
        action.addSubActionPair(robot.getIntakeShooter(), shoot, true);
        addAction(action) ;
    }

    protected void driveAndCollectAndShoot(String path, boolean setpose) throws Exception {

        AllegroRobot2024 robot = (AllegroRobot2024)getAutoController().getRobot().getRobotSubsystem() ;  

        SequenceAction seq = new SequenceAction(getAutoController().getRobot().getMessageLogger()) ;    
        SwerveHolonomicPathFollower pathact = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 0.2, mirror_, mvalue_);
        SwerveTrackAngle rotate = null ;

        start_collect_ = new StartCollectAltAction(robot.getIntakeShooter()) ;

        double collectLength = pathact.getDistance() - kCollectEndOfPathDistance ;        

        pathact.addDistanceBasedAction(collectLength, () -> { robot.getIntakeShooter().setAction(start_collect_); });
        seq.addSubActionPair(robot.getSwerve(), pathact, true) ;

        if (kRequireAprilTagAndRotate) {
            ISettingsSupplier settings = robot.getRobot().getSettingsSupplier() ;
            IntakeShooterSubsystem intake = robot.getIntakeShooter() ;
            double postol = intake.getSettingsValue("actions:auto-shoot:rotational-position-tolerance").getDouble() ;
            rotate = new SwerveTrackAngle(robot.getSwerve(), () -> robot.getTargetTracker().getRotation(), postol) ;
            addSubActionPair(robot.getSwerve(), rotate, false);
        }        

        IntakeAutoShootAction action = new IntakeAutoShootAction(robot.getIntakeShooter(), robot.getTargetTracker(), true, rotate) ;
        seq.addSubActionPair(robot.getIntakeShooter(), action, true);
        
        addAction(seq);
    }   
}
