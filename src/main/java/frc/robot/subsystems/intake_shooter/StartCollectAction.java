package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

//This begins the collecting process of moving the intake down to the right position and collects the game piece. This is not the only
//code and has a second part, FinishCollectAction. This uses the updown, tilt and feeder motors. This has a state machine which makes 
//tilt motor move first before the updown brings the intake down.

public class StartCollectAction extends Action{
 
    private enum State {
        Idle,
        TiltUp,
        IntakeDown,
        WaitForNote
    }

    private IntakeShooterSubsystem sub_;

    //Defining the collect actions for the different motors
    private MCMotionMagicAction collect_updown_;

    private MCMotionMagicAction collect_tilt_;

    private MCVelocityAction collect_feeder_;

    private State state_;

    public StartCollectAction(IntakeShooterSubsystem sub)throws Exception {
        super(sub.getRobot().getMessageLogger());
        //switches the state to Idle at first where it doewsn't do anything
        state_ = State.Idle;

        sub_ = sub;
        //Initializing the upDown and tilt collect actions with the Motion Magic constructor
        collect_updown_ = new MCMotionMagicAction(sub.getUpDown(), "pid:position", "targets:collect",0.5, 0.5);

        collect_tilt_ = new MCMotionMagicAction(sub.getTilt(), "pid:position", "targets:collect", 0,0 );
        //Initializing the feeder collect action with the velocity action
        collect_feeder_ = new MCVelocityAction(sub.getFeeder(), "pid:postion", "targets:collect");

    }
    @Override
    public void start() throws Exception {
        super.start();
        //Sets the state to where it begins to move the Tilt upwards to a safe position. Also begins spinning the feeder motor
        state_ = State.TiltUp;
        sub_.getFeeder().setAction(collect_feeder_, true);
   }
    @Override
    public void run() throws Exception{
        super.run();
       //Defines the sequence of each state and what to do in these states
        switch(state_) {
            case Idle:
                break;
            case TiltUp:
                stateTiltUp();
            case IntakeDown:
                stateIntakeDown();
            case WaitForNote:
                stateWaitForNote();
        }
   }
   @Override
    public String toString(int indent) {
        return prefix(indent) + "StartCollectAction";
    }
    private void stateTiltUp() {
       //Runs the tilt motor to the position we want and switches to the IntakeDown state so in the next robot loop, it will run that.
        sub_.getTilt().setAction(collect_tilt_, true);
        state_ = State.IntakeDown;
    }
    private void stateIntakeDown() {
        //This moves the upDown motor down to the desired position and continues moving the Tilt motor until it reaches it's final position 
        sub_.getUpDown().setAction(collect_updown_, true);

        sub_.getTilt().setAction(collect_tilt_, true);

        state_ = State.WaitForNote;
    }
    private void stateWaitForNote() {
        //This uses the sensor and keeps checking until the sensor detects a note is in place
        if (sub_.isNotePresent()) {
            sub_.getFeeder().cancelAction();
            setDone();
        }
    }

}
