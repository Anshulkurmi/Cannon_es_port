package math;

public class TransformOptions {
    protected Vec3 position;
    protected Quaternion quaternion;

    public TransformOptions() {
        // Default constructor
    	this.position = new Vec3();
    	this.quaternion = new Quaternion();
    }

    public Vec3 getPosition() {
        return position;
    }

    public void setPosition(Vec3 position) {
        this.position = position;
    }

    public Quaternion getQuaternion() {
        return quaternion;
    }

    public void setQuaternion(Quaternion quaternion) {
        this.quaternion = quaternion;
    }
}