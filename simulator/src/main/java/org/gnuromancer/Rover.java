package org.gnuromancer;

import simbad.sim.Agent;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

import javax.vecmath.Vector3d;

public class Rover extends Agent {

    private RangeSensorBelt sensorBelt;
    private double rotationalVelocityFactor = Math.PI / 32;

    public Rover(Vector3d position, String name) {
        super(position, name);

        sensorBelt = RobotFactory.addSonarBeltSensor(this, 1);
    }

    public void initBehavior() {
    }

    public void performBehavior() {
        if (sensorBelt.oneHasHit() || collisionDetected()) {
            double maxRotationalVelocity = Math.PI / 0.5;
            double distance = sensorBelt.getMeasurement(0);
            if (distance < 2 || Double.isInfinite(distance)) {

                // Randomly change direction to either left or right when collision is detected
                if ((int) (Math.random() * 101) % 2 == 0) {
                    setRotationalVelocity(maxRotationalVelocity - (rotationalVelocityFactor * Math.random()));
                    setTranslationalVelocity(0);
                } else {
                    setRotationalVelocity(-maxRotationalVelocity - (rotationalVelocityFactor * Math.random()));
                }
                setTranslationalVelocity(0);
            }
        } else {
            setRotationalVelocity(0);
            setTranslationalVelocity(1.0);
        }
    }
}