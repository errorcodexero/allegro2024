package frc.robot.automodes;

import org.xero1425.base.utils.Pose2dWithRotation;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class Start2Shoot4DynamicAction extends AllegroAutoModeAction {

    public enum FinishStrategy {
        Stop,
        NearSide,
        FarSide
    }

    private static final double kDelayForIntakeDownTime = 0.5 ;
    private static final double kPath2ShootDelay = 0.5 ;
    private static final double kPath4ShootDelay = 1.4 ;
    private static final double kPath6ShootDelay = 1.2 ;

    private static final double [] kPathMaxVelocity = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 } ;
    private static final double [] kPathMaxAccel = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 } ;

    private static final Pose2dWithRotation kShootPose = new Pose2dWithRotation(new Pose2d(1.46, 5.52, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(128.0)) ;
    private static final Pose2dWithRotation kCollect1Pose = new Pose2dWithRotation(new Pose2d(2.46, 5.32, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kCollect2Pose = new Pose2dWithRotation(new Pose2d(2.46, 6.32, Rotation2d.fromDegrees(45.0)), Rotation2d.fromDegrees(45.0)) ;
    private static final Pose2dWithRotation kCollect3Pose = new Pose2dWithRotation(new Pose2d(2.46, 4.07, Rotation2d.fromDegrees(-45.0)), Rotation2d.fromDegrees(-45.0)) ;
    private static final Pose2dWithRotation kNearSidePose = new Pose2dWithRotation(new Pose2d(7.42, 7.37, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kFarSidePose1 = new Pose2dWithRotation(new Pose2d(2.84, 2.63, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;
    private static final Pose2dWithRotation kFarSidePose2 = new Pose2dWithRotation(new Pose2d(7.42, 0.86, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0)) ;

    private enum State {
        Idle,
        Shoot1,
        DelayForIntakeDown,
        DrivingPath1,
        Path1Delay,
        DrivingPath2,
        DrivingPath3,
        DrivingPath4,
        DrivingPath5,
        DrivingPath6,
        DrivingPath7
    }

    private State state_ ;
    private double state_start_time_ ;
    private FinishStrategy finish_ ;

    public Start2Shoot4DynamicAction(AllegroRobot2024 robot, boolean mirror, double mvalue, FinishStrategy finish) throws Exception {
        super(robot, mirror, mvalue) ;
        state_ = State.Idle ;
        state_start_time_ = 0.0 ;
        finish_ = finish ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        state_ = State.Shoot1 ;
        state_start_time_ = getRobot().getTime() ;
        getRobotSubsystem().getIntakeShooter().setHoldingNote(true) ;
        getManualShootLowAction().setDelay(0.0) ;
        getRobotSubsystem().getIntakeShooter().setAction(getManualShootLowAction(), true) ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        State prev = state_ ;
        switch(state_) {
            case Shoot1:
                if (!getRobotSubsystem().getIntakeShooter().isHoldingNote()) {
                    state_ = State.DelayForIntakeDown ;
                    getRobotSubsystem().getIntakeShooter().setAction(getStartCollectAction(), true) ;
                }
                break ;

            case DelayForIntakeDown:
                if (getRobot().getTime() - state_start_time_ > kDelayForIntakeDownTime) {
                    gotoPoseWithRotationAndCollect("S2S4D-P1", kPathMaxVelocity[0], kPathMaxAccel[0], kCollect1Pose);
                    state_ = State.DrivingPath1 ;
                }
                break ;

            case DrivingPath1:
                if (isCurrentPathDone()) {
                    if (hasNote()) {
                        state_ = State.Path1Delay ;
                    }
                    else {
                        //
                        // TODO - test if it missing the note it does the right thing
                        //        or if we need more waypoints on the path between collect spots
                        //                        
                        gotoPoseWithRotationAndCollect("S2S4D-P3", kPathMaxVelocity[2], kPathMaxAccel[2], kCollect2Pose);
                        state_ = State.DrivingPath3 ;    
                    }                    
                }
                break ;
            case Path1Delay:
                if (getRobot().getTime() - state_start_time_ > 1.0) {
                    gotoPoseWithRotationAndShoot("S2S4D-P2", kPathMaxVelocity[1], kPathMaxAccel[1], kShootPose, kPath2ShootDelay);
                    state_ = State.DrivingPath2 ;                    
                }
                break ;
            
            case DrivingPath2:
                if (!hasNote()) {
                    gotoPoseWithRotationAndCollect("S2S4D-P3", kPathMaxVelocity[2], kPathMaxAccel[2], kCollect2Pose);
                    state_ = State.DrivingPath3 ;
                }
                break ;

            case DrivingPath3:
                if (isCurrentPathDone()) {
                    if (hasNote()) {
                        gotoPoseWithRotationAndShoot("S2S4D-P4", kPathMaxVelocity[3], kPathMaxAccel[3], kShootPose, kPath4ShootDelay);
                        state_ = State.DrivingPath4 ;
                    }
                    else {
                        //
                        // TODO - test if it missing the note it does the right thing
                        //        or if we need more waypoints on the path between collect spots
                        //                        
                        gotoPoseWithRotationAndCollect("S2S4D-P5", kPathMaxVelocity[4], kPathMaxAccel[4], kCollect3Pose);
                        state_ = State.DrivingPath5 ;
                    }
                }
                break ;

            case DrivingPath4:
                if (isCurrentPathDone() && !hasNote()) {
                    gotoPoseWithRotationAndCollect("S2S4D-P5", kPathMaxVelocity[4], kPathMaxAccel[4], kCollect3Pose);
                    state_ = State.DrivingPath5 ;
                }
                break ;

            case DrivingPath5:
                if (isCurrentPathDone()) {
                    gotoPoseWithRotationAndShoot("S2S4D-P6", kPathMaxVelocity[5], kPathMaxAccel[5], kShootPose, kPath6ShootDelay);
                    state_ = State.DrivingPath6 ;
                }
                break ;

            case DrivingPath6:
                if (isCurrentPathDone()) {
                    getRobotSubsystem().getSwerve().setAction(getStowAction(), true) ;
                    switch(finish_) {
                        case Stop:
                            break ;

                        case NearSide:
                            gotoPoseWithRotationAndCollect("S2S4D-P7", kPathMaxVelocity[6], kPathMaxAccel[6], kNearSidePose);
                            break ;

                        case FarSide:
                            gotoPoseWithRotation("S2S4D-P7", kPathMaxVelocity[6], kPathMaxAccel[6], new Pose2dWithRotation[] { kFarSidePose1, kFarSidePose2 });
                            break ;
                    }
                    state_ = State.Idle ;
                }

            default:
                break ;
        }

        if (prev != state_) {
            state_start_time_ = getRobot().getTime() ;            
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info) ;
            logger.add("Start2Shoot4DynamicAction : " + prev.toString() + " -> " + state_.toString()) ;
            logger.endMessage();
        }        
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "Start2Shoot4DynamicAction" ;
    }
}
