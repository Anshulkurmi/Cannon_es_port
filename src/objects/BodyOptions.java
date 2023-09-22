package objects;

import math.Vec3;
import material.Material;
import math.Quaternion;
import shapes.Shape;


public class BodyOptions {
    protected int collisionFilterGroup;
    protected int collisionFilterMask;
    protected boolean collisionResponse;
    protected Vec3 position;
    protected Vec3 velocity;
    protected double mass;
    protected Material material;
    protected double linearDamping;
    protected BodyTypes type;
    protected boolean allowSleep;
    protected double sleepSpeedLimit;
    protected double sleepTimeLimit;
    protected Quaternion quaternion;
    protected Vec3 angularVelocity;
    protected boolean fixedRotation;
    protected double angularDamping;
    protected Vec3 linearFactor;
    protected Vec3 angularFactor;
    protected Shape shape;
    protected boolean isTrigger;

    // Constructor
    public BodyOptions() {
        // Initialize with default values
        collisionFilterGroup = 1;
        collisionFilterMask = -1;
        collisionResponse = true;
        position = new Vec3();
        velocity = new Vec3();
        mass = 0;
        material = null;
        linearDamping = 0.01;
        type = BodyTypes.DYNAMIC;
        allowSleep = true;
        sleepSpeedLimit = 0.1;
        sleepTimeLimit = 1;
        quaternion = new Quaternion();
        angularVelocity = new Vec3();
        fixedRotation = false;
        angularDamping = 0.01;
        linearFactor = new Vec3(1, 1, 1);
        angularFactor = new Vec3(1, 1, 1);
        shape = null;
        isTrigger = false;
    }

    // Add getter and setter methods for each field...

    public int getCollisionFilterGroup() {
        return collisionFilterGroup;
    }

    public void setCollisionFilterGroup(int collisionFilterGroup) {
        this.collisionFilterGroup = collisionFilterGroup;
    }

    public int getCollisionFilterMask() {
        return collisionFilterMask;
    }

    public void setCollisionFilterMask(int collisionFilterMask) {
        this.collisionFilterMask = collisionFilterMask;
    }

    // Add getters and setters for the remaining fields...
}
