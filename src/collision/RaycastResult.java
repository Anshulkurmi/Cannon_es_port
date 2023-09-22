package collision;

import math.Vec3;
import objects.Body;
import shapes.Shape;
/**
 * Storage for Ray casting data
 */
public class RaycastResult {
    /**
     * rayFromWorld
     */
    public Vec3 rayFromWorld;
    /**
     * rayToWorld
     */
    public Vec3 rayToWorld;
    /**
     * hitNormalWorld
     */
    public Vec3 hitNormalWorld;
    /**
     * hitPointWorld
     */
    public Vec3 hitPointWorld;
    /**
     * hasHit
     */
    public boolean hasHit;
    /**
     * shape
     */
    public Shape shape;
    /**
     * body
     */
    public Body body;
    /**
     * The index of the hit triangle, if the hit shape was a trimesh
     */
    public int hitFaceIndex;
    /**
     * Distance to the hit. Will be set to -1 if there was no hit
     */
    public double distance;
    /**
     * If the ray should stop traversing the bodies
     */
    public boolean shouldStop;

    public RaycastResult() {
        this.rayFromWorld = new Vec3();
        this.rayToWorld = new Vec3();
        this.hitNormalWorld = new Vec3();
        this.hitPointWorld = new Vec3();
        this.hasHit = false;
        this.shape = null;
        this.body = null;
        this.hitFaceIndex = -1;
        this.distance = -1;
        this.shouldStop = false;
    }

    /**
     * Reset all result data.
     */
    public void reset() {
        this.rayFromWorld.setZero();
        this.rayToWorld.setZero();
        this.hitNormalWorld.setZero();
        this.hitPointWorld.setZero();
        this.hasHit = false;
        this.shape = null;
        this.body = null;
        this.hitFaceIndex = -1;
        this.distance = -1;
        this.shouldStop = false;
    }

    /**
     * Abort the raycasting process.
     */
    public void abort() {
        this.shouldStop = true;
    }

    /**
     * Set result data.
     */
    public void set(
        Vec3 rayFromWorld,
        Vec3 rayToWorld,
        Vec3 hitNormalWorld,
        Vec3 hitPointWorld,
        Shape shape,
        Body body,
        double distance
    ) {
        this.rayFromWorld.copy(rayFromWorld);
        this.rayToWorld.copy(rayToWorld);
        this.hitNormalWorld.copy(hitNormalWorld);
        this.hitPointWorld.copy(hitPointWorld);
        this.shape = shape;
        this.body = body;
        this.distance = distance;
    }
}
