package org.gnuromancer;

import simbad.gui.Simbad;
import simbad.sim.EnvironmentDescription;

import javax.vecmath.Vector3d;

public class RoverSim {

    private final static int ROVER_COUNT = 3;

    public static class Space extends EnvironmentDescription {
        public Space(int roverCount) {
            EnvironmentFactory.createSquareSpace(16, this);

            for (int cnt = 0; cnt < roverCount; cnt++) {
                add(new Rover(new Vector3d(0, 0, 0), "rover " + cnt));
            }
        }
    }

    public static void main(String[] args) {
        int roverCount = ROVER_COUNT;

        try {
            if (args.length > 0) {
                roverCount = Integer.parseInt(args[0]);
            }
        } catch (NumberFormatException nfe) {
            System.err.println("Invalid argument, defaulting to " + ROVER_COUNT);
        }

        Simbad simbad = new Simbad(new Space(roverCount), false);
    }
}