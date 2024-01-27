package frc.robot.subsystems.ampTrap;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class AmpTrapSubsystem extends Subsystem{
    private MotorEncoderSubsystem elevator_;
    private MotorEncoderSubsystem arm_;
    private MotorEncoderSubsystem wrist_;
    private MotorEncoderSubsystem manipulator_;
    private MotorEncoderSubsystem climber_;

    public AmpTrapSubsystem(Subsystem parent) throws Exception{
        super(parent, "ampTrap");

        elevator_ = new MotorEncoderSubsystem(this, "elevator", false);
        //elevator_.getMotorController().setCurrentLimit(20.0);
        addChild(elevator_);


        arm_ = new MotorEncoderSubsystem(this, "arm", true);
        //arm_.getMotorController().setCurrentLimit(20.0);
        addChild(arm_);

        wrist_ = new MotorEncoderSubsystem(this, "wrist", true);

        //wrist_.getMotorController().setCurrentLimit(20.0);
        addChild(wrist_);

        manipulator_ = new MotorEncoderSubsystem(this, "manipulator", false);
        //manipulator_.getMotorController().setCurrentLimit(20.0);
        addChild(manipulator_);

        climber_ = new MotorEncoderSubsystem(this, "climber", false);
        //climber_.getMotorController().setCurrentLimit(20.0);
        addChild(climber_);
    }

    public MotorEncoderSubsystem elevator(){
        return elevator_;
    }

    public MotorEncoderSubsystem arm(){
        return arm_;
    }

    public MotorEncoderSubsystem wrist(){
        return wrist_;
    }

    public MotorEncoderSubsystem manipulator(){
        return manipulator_;
    }

    public MotorEncoderSubsystem climber(){
        return climber_;
    }


}
