package org.xero1425.simulator.models;

import org.xero1425.simulator.engine.SimulationEngine;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import org.xero1425.base.motors.IMotorController;
import org.xero1425.base.motors.SparkFlexMotorController;
import org.xero1425.base.motors.SparkMaxMotorController;

public class SparkFlexSimMotorController extends SimMotorController {
    private static final double kTicksPerRev = 2048 ;
    private SparkFlexMotorController motor_ ;
    private DCMotor dcmotor_ ;
    private DCMotorSim sim_ ;
    private double gearing_ ;
    
    public SparkFlexSimMotorController(SimulationEngine engine, String bus, int canid, int count, double moment, double gearning) throws Exception {
        super(engine, bus, canid) ;

        if (!bus.isEmpty()) {
            throw new Exception("SparkMax controllers must have an empty bus");
        }

        IMotorController ctrl = engine.getRobot().getMotorFactory().getMotorController(bus, canid) ;
        if (!(ctrl instanceof SparkFlexMotorController)) {
            throw new Exception("motor on bus '" + bus + "', can id " + canid + " is not a CANSparkFlex motor");
        }

        motor_ = (SparkFlexMotorController)ctrl ;
        dcmotor_ = DCMotor.getNEO(count) ;
        sim_ = new DCMotorSim(dcmotor_, 1.0, moment);
    }

    @Override
    public void run(double dt) {
        SparkFlexMotorController.SimState state = motor_.getSimState() ;

        sim_.setInputVoltage(state.getMotorVoltage());

        double pos = sim_.getAngularPositionRotations() * kTicksPerRev * gearing_ ;
        double vel = sim_.getAngularVelocityRPM() * 60.0 * kTicksPerRev * gearing_ ;
        state.setPosition(pos) ;
        state.setVelocity(vel) ;

        addPlotData(12.0, state.getMotorVoltage(), pos, vel) ;
    }

    @Override
    public double ticksPerRev() {
        return kTicksPerRev ;
    }

    @Override
    public double voltage() {
        SparkFlexMotorController.SimState state = motor_.getSimState() ;
        return state.getMotorVoltage() ;        
    }

    @Override
    public double position() {
        return sim_.getAngularPositionRotations() * kTicksPerRev * gearing_ ;
    }    

    @Override
    public double velocity() {
        return sim_.getAngularVelocityRPM() * 60.0 * kTicksPerRev * gearing_ ;    
    }       
} ;
