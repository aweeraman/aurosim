package org.gnuromancer;

import simbad.sim.EnvironmentDescription;
import simbad.sim.Wall;

import javax.vecmath.Vector3d;

/**
 * Created by anuradha on 5/16/16.
 */
public class EnvironmentFactory {

    public static void createSquareSpace(int length, EnvironmentDescription env) {
        Wall w1 = new Wall(new Vector3d(length / 2, 0, 0), length, 2, env);
        w1.rotate90(1);

        Wall w2 = new Wall(new Vector3d(-length / 2, 0, 0), length, 2, env);
        w2.rotate90(1);

        Wall w3 = new Wall(new Vector3d(0, 0, length / 2), length, 2, env);

        Wall w4 = new Wall(new Vector3d(0, 0, -length / 2), length, 2, env);

        env.add(w1);
        env.add(w2);
        env.add(w3);
        env.add(w4);
    }
}
