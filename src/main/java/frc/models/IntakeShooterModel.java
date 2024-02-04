package frc.models;

import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.ISimMotorController;

public class IntakeShooterModel extends SimulationModel {
    final private static String bus = "" ;
    final private static int [] RequiredMotors = new int[] { 1, 2, 3, 4, 5} ;

    ISimMotorController updown_ ;
    ISimMotorController feeder_ ;
    ISimMotorController shooter1_ ;
    ISimMotorController shooter2_ ;    
    ISimMotorController tilt_ ;
    
    public IntakeShooterModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create(SimulationEngine engine) throws Exception {
        if (!hasCANMotors(bus, RequiredMotors)) {
            return false ;
        }

        //
        // Create the models for the motors
        //
        feeder_ = createSimulatedMotor(engine, "feeder");
        updown_ = createSimulatedMotor(engine, "updown");
        feeder_ = createSimulatedMotor(engine, "feeder");
        shooter1_ = createSimulatedMotor(engine, "shooter1");
        shooter2_ = createSimulatedMotor(engine, "shooter2");
        tilt_ = createSimulatedMotor(engine, "tilt");
        

        setCreated();

        return true;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        return false ;
    }

    @Override
    public void run(double dt) {
        if (feeder_ != null) {
            feeder_.run(dt) ;
            updown_.run(dt) ;
            shooter1_.run(dt) ;
            shooter2_.run(dt) ;        
            tilt_.run(dt) ;
        }
    }
}
