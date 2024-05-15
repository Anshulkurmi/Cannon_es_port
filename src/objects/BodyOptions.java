package objects;

import math.Vec3;
import material.Material;
import math.Quaternion;
import shapes.Box;
import shapes.Shape;
import shapes.ShapeOptions;
import shapes.Sphere;


public class BodyOptions extends ShapeOptions {
    protected int collisionFilterGroup;
    protected int collisionFilterMask;
    protected boolean collisionResponse;
    protected Vec3 position;
    
	protected Vec3 velocity;
    private double mass;
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
    private Shape shape;
    protected boolean isTrigger;

    // Constructor
    public BodyOptions() {
        // Initialize with default values
        collisionFilterGroup = 1;
        collisionFilterMask = -1;
        collisionResponse = true;
        position = new Vec3();
        velocity = new Vec3();
        setMass(0);
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
        setShape(null);
        isTrigger = false;
    }
    
    public BodyOptions(double mass , Shape shape) {
    	this(1,-1,true,new Vec3(),new Vec3(),mass,null,0.01,BodyTypes.DYNAMIC,true , 0.1,1,new Quaternion(),new Vec3(),false , 0.01,new Vec3(1, 1, 1),new Vec3(1, 1, 1),shape,false);
    }
    
    public BodyOptions(double mass , Sphere sphere) {
    	this(1,-1,true,new Vec3(),new Vec3(),mass,null,0.01,BodyTypes.DYNAMIC,true , 0.1,1,new Quaternion(),new Vec3(),false , 0.01,new Vec3(1, 1, 1),new Vec3(1, 1, 1),sphere,false);
    }
    
    public BodyOptions(double mass , Box box) {
    	this(1,-1,true,new Vec3(),new Vec3(),mass,null,0.01,BodyTypes.DYNAMIC,true , 0.1,1,new Quaternion(),new Vec3(),false , 0.01,new Vec3(1, 1, 1),new Vec3(1, 1, 1),box,false);
    }
    
    public BodyOptions(int collisionFilterGroup,int collisionFilterMask,boolean collisionResponse,Vec3 position, Vec3 velocity,
     double mass,
     Material material ,
     double linearDamping ,
     BodyTypes type ,
     boolean allowSleep,
     double sleepSpeedLimit,
     double sleepTimeLimit,
     Quaternion quaternion,
     Vec3 angularVelocity,
     boolean fixedRotation,
     double angularDamping,
     Vec3 linearFactor,
     Vec3 angularFactor,
     Shape shape,
     boolean isTrigger) {
    	this.collisionFilterGroup = collisionFilterGroup;
        this.collisionFilterMask = collisionFilterMask;
        this.collisionResponse = collisionResponse;
        this.position = position;
        this.velocity = velocity;
        this.setMass(mass);
        this.material = material;
        this.linearDamping = linearDamping;
        this.type = type;
        this.allowSleep = allowSleep;
        this.sleepSpeedLimit = sleepSpeedLimit;
        this.sleepTimeLimit = sleepTimeLimit;
        this.quaternion = quaternion;
        this.angularVelocity = angularVelocity;
        this.fixedRotation = fixedRotation;
        this.angularDamping = angularDamping;
        this.linearFactor = linearFactor;
        this.angularFactor = angularFactor;
        this.setShape(shape);
        this.isTrigger = isTrigger; 	
    }

    // Add getter and setter methods for each field...

    public int getCollisionFilterGroup() {
        return this.collisionFilterGroup;
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

	public Shape getShape() {
		return shape;
	}

	public void setShape(Shape shape) {
		this.shape = shape;
	}

	public double getMass() {
		return mass;
	}

	public void setMass(double mass) {
		this.mass = mass;
	}
    
	public boolean isCollisionResponse() {
		return collisionResponse;
	}

	public void setCollisionResponse(boolean collisionResponse) {
		this.collisionResponse = collisionResponse;
	}

	public Vec3 getPosition() {
		return position;
	}

	public void setPosition(Vec3 position) {
		this.position = position;
	}

	public Vec3 getVelocity() {
		return velocity;
	}

	public void setVelocity(Vec3 velocity) {
		this.velocity = velocity;
	}

	public Material getMaterial() {
		return material;
	}

	public void setMaterial(Material material) {
		this.material = material;
	}

	public double getLinearDamping() {
		return linearDamping;
	}

	public void setLinearDamping(double linearDamping) {
		this.linearDamping = linearDamping;
	}

	public BodyTypes getType() {
		return type;
	}

	public void setType(BodyTypes type) {
		this.type = type;
	}

	public boolean isAllowSleep() {
		return allowSleep;
	}

	public void setAllowSleep(boolean allowSleep) {
		this.allowSleep = allowSleep;
	}

	public double getSleepSpeedLimit() {
		return sleepSpeedLimit;
	}

	public void setSleepSpeedLimit(double sleepSpeedLimit) {
		this.sleepSpeedLimit = sleepSpeedLimit;
	}

	public double getSleepTimeLimit() {
		return sleepTimeLimit;
	}

	public void setSleepTimeLimit(double sleepTimeLimit) {
		this.sleepTimeLimit = sleepTimeLimit;
	}

	public Quaternion getQuaternion() {
		return quaternion;
	}

	public void setQuaternion(Quaternion quaternion) {
		this.quaternion = quaternion;
	}

	public Vec3 getAngularVelocity() {
		return angularVelocity;
	}

	public void setAngularVelocity(Vec3 angularVelocity) {
		this.angularVelocity = angularVelocity;
	}

	public boolean isFixedRotation() {
		return fixedRotation;
	}

	public void setFixedRotation(boolean fixedRotation) {
		this.fixedRotation = fixedRotation;
	}

	public double getAngularDamping() {
		return angularDamping;
	}

	public void setAngularDamping(double angularDamping) {
		this.angularDamping = angularDamping;
	}

	public Vec3 getLinearFactor() {
		return linearFactor;
	}

	public void setLinearFactor(Vec3 linearFactor) {
		this.linearFactor = linearFactor;
	}

	public Vec3 getAngularFactor() {
		return angularFactor;
	}

	public void setAngularFactor(Vec3 angularFactor) {
		this.angularFactor = angularFactor;
	}

	public boolean isTrigger() {
		return isTrigger;
	}

	public void setTrigger(boolean isTrigger) {
		this.isTrigger = isTrigger;
	}

    
    // Add getters and setters for the remaining fields...
}
