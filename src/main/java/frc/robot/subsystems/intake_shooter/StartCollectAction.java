package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.actions.Action;

public class StartCollectAction extends Action{
    // Great start pranay! There are a couple of things that should happen with this action though. 
    private IntakeShooterSubsystem sub_;

    //The collect action
    private MCMotionMagicAction collect_updown_;

    //You need to make a way for the tilt motor to move too. We can't just rely on the position that it was in before!

    //The feeder motors need a way to spin. This action should be an MCVelocityAction. 


    public StartCollectAction(IntakeShooterSubsystem sub)throws Exception{
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        collect_updown_ = new MCMotionMagicAction(sub.getUpDown(), "pid:position", "targets:collect",0.5, 0.5);
        //Initialize the other actions here. 

        //For the MCVelocityAction, you can initialize them basically the same as I did in my PlaceAction. Just change the motor and the target.

    }
   @Override
   public void start() throws Exception {
       super.start();
       sub_.getUpDown().setAction(collect_updown_, true);
       //Set the other actions here, just like you did above

       //I would not set the velocity action here, because we want them to start when the updown and tilt motors have put the intake in place. 
   }
   @Override
   public void run() throws Exception{
       super.run();
       //Check if the updown and tilt motors are done, and if they are, start spinning the spinner motors and set the action as done. 

       //Or, if you want, do not set the action as done and check if the NoteSensor is triggered. When that happens, then you can write new actions for raising the shooter back and stopping the spinner motors. 
       //Then when those are done, you can do the setDone, consolitating the startCollect and stopCollect into one action. 
       //You do not have to do this option, it is just another idea that you could use when finishing up this action. 
   }
   
    @Override
    public String toString(int indent) {
        return prefix(indent) + "StartCollectAction";
    }

}
