package utils; 

import math.Vec3; 

/**
 * Vec3Pool
 */
public class Vec3Pool extends Pool {
    //private Class<Vec3> type = Vec3.class;

    /**
     * Construct a vector
     */
    @Override
    protected Vec3 constructObject() {
        return new Vec3();
    }
}
