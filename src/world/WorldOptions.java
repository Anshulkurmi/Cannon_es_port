package world;

import collision.Broadphase;
import math.Vec3;
import solver.Solver;

public class WorldOptions {
    /**
       * The gravity of the world.
       */
    protected Vec3 gravity;
    /**
       * Gravity to use when approximating the friction max force (mu*mass*gravity).
       * If undefined, global gravity will be used.
       */
    protected Vec3 frictionGravity;
    /**
       * Makes bodies go to sleep when they've been inactive.
       * @default false
       */
    protected boolean allowSleep;
    /**
       * The broadphase algorithm to use.
       * @default NaiveBroadphase
       */
    protected Broadphase broadphase;
    /**
       * The solver algorithm to use.
       * @default GSSolver
       */
    protected Solver solver;
    /**
       * Set to true to use fast quaternion normalization. It is often enough accurate to use.
       * If bodies tend to explode, set to false.
       * @default false
       */
    protected boolean quatNormalizeFast;
    /**
       * How often to normalize quaternions. Set to 0 for every step, 1 for every second etc.. A larger value increases performance. If bodies tend to explode, set to a smaller value (zero to be sure nothing can go wrong).
       * @default 0
       */
    protected int quatNormalizeSkip;

    // public WorldOptions(){
    //     this.solver = new GSSolver();
    //     this.gravity = 
    // }

    public Vec3 getGravity() {
        return gravity;
    }

    public void setGravity(Vec3 gravity) {
        this.gravity = gravity;
    }

    public Vec3 getFrictionGravity() {
        return frictionGravity;
    }

    public void setFrictionGravity(Vec3 frictionGravity) {
        this.frictionGravity = frictionGravity;
    }

    public boolean isAllowSleep() {
        return allowSleep;
    }

    public void setAllowSleep(boolean allowSleep) {
        this.allowSleep = allowSleep;
    }

    public Broadphase getBroadphase() {
        return broadphase;
    }

    public void setBroadphase(Broadphase broadphase) {
        this.broadphase = broadphase;
    }

    public Solver getSolver() {
        return solver;
    }

    public void setSolver(Solver solver) {
        this.solver = solver;
    }

    public boolean isQuatNormalizeFast() {
        return quatNormalizeFast;
    }

    public void setQuatNormalizeFast(boolean quatNormalizeFast) {
        this.quatNormalizeFast = quatNormalizeFast;
    }

    public int getQuatNormalizeSkip() {
        return quatNormalizeSkip;
    }

    public void setQuatNormalizeSkip(int quatNormalizeSkip) {
        this.quatNormalizeSkip = quatNormalizeSkip;
    }
}
