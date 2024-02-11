package frc.robot.automodes;

import org.xero1425.base.actions.ParallelAction;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.actions.ParallelAction.DonePolicy;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.subsystems.swerve.SwerveHolonomicPathFollower;

import frc.robot.subsystems.intake_shooter.ButchStartCollectAction;
import frc.robot.subsystems.intake_shooter.IntakeAutoShootAction;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class AllegroGameAutoMode extends AutoMode {
    private final static double kCollectEndOfPathDistance = 0.5 ;
    private final static double kShootEndOfPathDistance = 0.08 ;

    private ButchStartCollectAction start_collect_ ;

    public AllegroGameAutoMode(AutoController ctrl, String name) {
        super(ctrl, name) ;
    }

    protected void shootFirstNote() throws Exception {
        AllegroRobot2024 robot = (AllegroRobot2024)getAutoController().getRobot().getRobotSubsystem() ;

        IntakeAutoShootAction action = new IntakeAutoShootAction(robot.getIntakeShooter(), robot.getTargetTracker(), true) ;
        addSubActionPair(robot.getIntakeShooter(), action, true);
    }

    protected void driveAndCollect(String path, boolean setpose) throws Exception {
        AllegroRobot2024 robot = (AllegroRobot2024)getAutoController().getRobot().getRobotSubsystem() ;        
        SwerveHolonomicPathFollower pathact = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 0.2);
        start_collect_ = new ButchStartCollectAction(robot.getIntakeShooter()) ;

        double collectLength = pathact.getDistance() - kCollectEndOfPathDistance ;

        pathact.addDistanceBasedAction(collectLength, () -> { robot.getIntakeShooter().setAction(start_collect_); });
        addSubActionPair(robot.getSwerve(), pathact, true) ;
    }

    protected void driveAndShoot(String path, boolean setpose) throws Exception {
        AllegroRobot2024 robot = (AllegroRobot2024)getAutoController().getRobot().getRobotSubsystem() ;        
        SwerveHolonomicPathFollower pathact = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 0.2);
        IntakeAutoShootAction shoot = new IntakeAutoShootAction(robot.getIntakeShooter(), robot.getTargetTracker(), false) ;

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
        SwerveHolonomicPathFollower pathact = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 0.2);
        start_collect_ = new ButchStartCollectAction(robot.getIntakeShooter()) ;

        double collectLength = pathact.getDistance() - kCollectEndOfPathDistance ;        

        pathact.addDistanceBasedAction(collectLength, () -> { robot.getIntakeShooter().setAction(start_collect_); });
        seq.addSubActionPair(robot.getSwerve(), pathact, true) ;

        IntakeAutoShootAction action = new IntakeAutoShootAction(robot.getIntakeShooter(), robot.getTargetTracker(), true) ;
        seq.addSubActionPair(robot.getIntakeShooter(), action, true);
    }   
}
