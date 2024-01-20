package frc.models;

import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.ISimMotorController;

public class IntakeShooterModel extends SimulationModel {
    ISimMotorController spinner_ ;
    ISimMotorController updown_ ;
    ISimMotorController feeder_ ;
    ISimMotorController shooter_ ;
    ISimMotorController tilt_ ;
    
    public IntakeShooterModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create(SimulationEngine engine) throws Exception {        
        spinner_ = createSimulatedMotor(engine, "spinner");
        updown_ = createSimulatedMotor(engine, "updown");
        feeder_ = createSimulatedMotor(engine, "feeder");
        shooter_ = createSimulatedMotor(engine, "shooter");
        tilt_ = createSimulatedMotor(engine, "tilt");

        setCreated();

        return false;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        return false ;
    }

    @Override
    public void run(double dt) {
        spinner_.run(dt) ;
        updown_.run(dt) ;
        feeder_.run(dt) ;
        shooter_.run(dt) ;
        tilt_.run(dt) ;
    }
}
