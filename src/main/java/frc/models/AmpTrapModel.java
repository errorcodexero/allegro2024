package frc.models;

import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.ISimMotorController;

public class AmpTrapModel extends SimulationModel {
    final private static String bus = "" ;
    final private static int [] RequiredMotors = new int[] { 6, 7, 8, 9, 10 } ;

    private ISimMotorController elevator_;
    private ISimMotorController arm_;
    private ISimMotorController wrist_;
    private ISimMotorController manipulator_;
    private ISimMotorController climber_;    

    public AmpTrapModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create(SimulationEngine engine) throws Exception 
    {
        if (!hasCANMotors(bus, RequiredMotors)) {
            return false ;
        }

        elevator_ = createSimulatedMotor(engine, "elevator");
        arm_ = createSimulatedMotor(engine, "arm");
        wrist_ = createSimulatedMotor(engine, "wrist");
        manipulator_ = createSimulatedMotor(engine, "manipulator");
        climber_ = createSimulatedMotor(engine, "climber");

        setCreated();        

        return true ;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        return false ;
    }

    @Override
    public void run(double dt) {
        if (elevator_ != null) {
            elevator_.run(dt) ;
            arm_.run(dt) ;
            wrist_.run(dt) ;    
            manipulator_.run(dt) ;  
            climber_.run(dt) ;
        }
    }
}
