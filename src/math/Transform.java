package math;


/**
 * Transformation utilities.
 */
public class Transform {
    /**
     * Position vector.
     */
    public Vec3 position;
    /**
     * Quaternion representing rotation.
     */
    public Quaternion quaternion;
    
    
    public Transform() {
    	this(new TransformOptions());
    }
    
    /**
     * Constructor for Transform class.
     *
     * @param options Options for initialization.
     */
    public Transform(TransformOptions options) {
        this.position = new Vec3();
        this.quaternion = new Quaternion();

        // Initialize with provided position if available.
        if (options.position != null) {
            this.position.copy(options.position);
        }

        // Initialize with provided quaternion if available.
        if (options.quaternion != null) {
            this.quaternion.copy(options.quaternion);
        }
    }

    /**
     * Get a global point in local transform coordinates.
     *
     * @param worldPoint Global point.
     * @param result     Resulting local point.
     * @return Local point.
     */
    public Vec3 pointToLocal(Vec3 worldPoint, Vec3 result) {
        return Transform.pointToLocalFrame(this.position, this.quaternion, worldPoint, result);
    }

    /**
     * Get a local point in global transform coordinates.
     *
     * @param localPoint Local point.
     * @param result     Resulting global point.
     * @return Global point.
     */
    public Vec3 pointToWorld(Vec3 localPoint, Vec3 result) {
        return Transform.pointToWorldFrame(this.position, this.quaternion, localPoint, result);
    }

    /**
     * Convert a local vector to global transform coordinates.
     *
     * @param localVector Local vector.
     * @param result      Resulting global vector.
     * @return Global vector.
     */
    public Vec3 vectorToWorldFrame(Vec3 localVector, Vec3 result) {
        this.quaternion.vmult(localVector, result);
        return result;
    }

    /**
     * Convert a global point to local transform coordinates.
     *
     * @param position   Global position.
     * @param quaternion Global rotation quaternion.
     * @param worldPoint Global point to convert.
     * @param result     Resulting local point.
     * @return Local point.
     */
    public static Vec3 pointToLocalFrame(Vec3 position, Quaternion quaternion, Vec3 worldPoint, Vec3 result) {
        // Translate the world point to local coordinates by subtracting the position vector.
        worldPoint.vsub(position, result);
        
        // Conjugate the quaternion (representing the inverse rotation) to rotate back to the local frame.
        //Quaternion tmpQuat = new Quaternion();
        quaternion.conjugate(tmpQuat);
        tmpQuat.vmult(result, result);
        
        return result;
    }

    /**
     * Convert a local point to global transform coordinates.
     *
     * @param position   Global position.
     * @param quaternion Global rotation quaternion.
     * @param localPoint Local point to convert.
     * @param result     Resulting global point.
     * @return Global point.
     */
    public static Vec3 pointToWorldFrame(Vec3 position, Quaternion quaternion, Vec3 localPoint, Vec3 result) {
        // Rotate the local point using the global rotation quaternion.
        quaternion.vmult(localPoint, result);

        // Translate the result by adding the global position vector.
        result.vadd(position);

        return result;
    }

    /**
     * Convert a local vector to global transform coordinates using a quaternion.
     *
     * @param quaternion   Global rotation quaternion.
     * @param localVector  Local vector to convert.
     * @param result       Resulting global vector.
     * @return Global vector.
     */
    public static Vec3 vectorToWorldFrame(Quaternion quaternion, Vec3 localVector, Vec3 result) {
        quaternion.vmult(localVector, result);
        return result;
    }

    /**
     * Convert a global vector to local transform coordinates using a quaternion.
     *
     * @param position    Global position.
     * @param quaternion  Global rotation quaternion.
     * @param worldVector Global vector to convert.
     * @param result      Resulting local vector.
     * @return Local vector.
     */
    public static Vec3 vectorToLocalFrame(Vec3 position, Quaternion quaternion, Vec3 worldVector, Vec3 result) {
        // Negate the scalar (w) component of the quaternion to get its conjugate.
        quaternion.w *= -1;

        // Rotate the global vector using the conjugate of the global rotation quaternion.
        quaternion.vmult(worldVector, result);

        // Restore the original quaternion by negating the scalar (w) component again.
        quaternion.w *= -1;

        return result;
    }

    static Quaternion tmpQuat = new Quaternion();
    
}
