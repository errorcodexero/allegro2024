package frc.robot.subsystems.intake;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.IMotorController;
import org.xero1425.base.motors.IMotorController.XeroPidType;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.SettingsValue;

public class IntakeSubsystem extends MotorEncoderSubsystem {
    private IMotorController spinner_ ;
    private double spinner_power_ ;

    public IntakeSubsystem(Subsystem parent) throws Exception {
        super(parent, "intake", false);

        String key = "subsystems:" + getName() + ":hw:spinner" ;
        spinner_ = getRobot().getMotorFactory().createMotor("intake-spinner", key) ;

        spinner_power_ = 0.0 ;
        spinner_.set(XeroPidType.Power, 0.0) ;

        getMotorController().enableVoltageCompensation(true, 11.0);
    }

    public void setSpinnerPower(double power) throws BadMotorRequestException, MotorRequestFailedException {
        spinner_.set(XeroPidType.Power, power); 
    }

    public double getSpinnerPower() {
        return spinner_power_ ;
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("spinner-power")) {
            v = new SettingsValue(spinner_power_);
        }

        return v ;
    }    
}
