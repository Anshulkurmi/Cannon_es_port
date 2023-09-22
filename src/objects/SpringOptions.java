package objects;

import math.Vec3;

public class SpringOptions {
    protected double restLength;
    protected double stiffness;
    protected double damping;
    protected Vec3 localAnchorA;
    protected Vec3 localAnchorB;
    protected Vec3 worldAnchorA;
    protected Vec3 worldAnchorB;

    public SpringOptions() {
        // Default values
        this.restLength = 1.0;
        this.stiffness = 100.0;
        this.damping = 1.0;
        this.localAnchorA = new Vec3();
        this.localAnchorB = new Vec3();
        this.worldAnchorA = null;
        this.worldAnchorB = null;
    }

    // Getters and setters for each property

    public Double getRestLength() {
        return restLength;
    }

    public void setRestLength(Double restLength) {
        this.restLength = restLength;
    }

    public Double getStiffness() {
        return stiffness;
    }

    public void setStiffness(Double stiffness) {
        this.stiffness = stiffness;
    }

    public Double getDamping() {
        return damping;
    }

    public void setDamping(Double damping) {
        this.damping = damping;
    }

    public Vec3 getLocalAnchorA() {
        return localAnchorA;
    }

    public void setLocalAnchorA(Vec3 localAnchorA) {
        this.localAnchorA = localAnchorA;
    }

    public Vec3 getLocalAnchorB() {
        return localAnchorB;
    }

    public void setLocalAnchorB(Vec3 localAnchorB) {
        this.localAnchorB = localAnchorB;
    }

    public Vec3 getWorldAnchorA() {
        return worldAnchorA;
    }

    public void setWorldAnchorA(Vec3 worldAnchorA) {
        this.worldAnchorA = worldAnchorA;
    }

    public Vec3 getWorldAnchorB() {
        return worldAnchorB;
    }

    public void setWorldAnchorB(Vec3 worldAnchorB) {
        this.worldAnchorB = worldAnchorB;
    }
}

