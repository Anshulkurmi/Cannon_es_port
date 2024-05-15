package math;

//import math.Vec3;

/**
 * A Quaternion describes a rotation in 3D space.
 * The Quaternion is mathematically defined as Q = x*i + y*j + z*k + w,
 * where (i,j,k) are imaginary basis vectors. 
 * (x,y,z) can be seen as a vector related to the axis of rotation,
 *  while the real multiplier, w, is related to the amount of rotation.
 *  
 *  for more infor on Quaternion , visit https://www.3dgep.com/understanding-quaternions/
 *  
 * @param x Multiplier of the imaginary basis vector i.
 * @param y Multiplier of the imaginary basis vector j.
 * @param z Multiplier of the imaginary basis vector k.
 * @param w Multiplier of the real part.
 * @see http://en.wikipedia.org/wiki/Quaternion
 */
public class Quaternion {
	public double x;
    public double y;
    public double z;
    public double w;
    
    public static double default_precision=1e-6;
    

    /**
     * Creates new Quaternion (1+0i+0j+0k)
     */
    public Quaternion() {
        this(0, 0, 0, 1);
    }

    /**
     * Creates new Quaternion (w+xi+yj+zk)
     * @param x
     * @param y
     * @param z
     * @param w
     */
    public Quaternion(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }
    
    /**
     * Set the value of the quaternion to (w+xi+yj+zk)
     */
    public Quaternion set(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;
    }
    
    /**
     * Convert to a readable format
     * @return "x,y,z,w"
     */
    @Override
    public String toString() {
        return this.x + "," + this.y + "," + this.z + "," + this.w;
    }
    
    /**
     * Convert to an Array
     * @return [x, y, z, w]
     */
    public double[] toArray() {
        return new double[] { this.x, this.y, this.z, this.w };
    }
    
    /**
     * Set the quaternion components given an axis(in vector) and an angle in radians.
     * @param vector : 3D vector xi + yj + zk ;
     * @param angle : angle to be set in the resulting quaternion 
     */
    public Quaternion setFromAxisAngle(Vec3 vector, double angle) {
        double s = Math.sin(angle * 0.5);
        this.x = vector.x * s;
        this.y = vector.y * s;
        this.z = vector.z * s;
        this.w = Math.cos(angle * 0.5);
        return this;
    }
    
    /**
     * default case of creating a quaternion to [axis,angle] if no targetAxis given 
     * @return
     */
    public Object[] toAxisAngle() {
    	return toAxisAngle(new Vec3());
    }
    
    /**
     * Converts the quaternion to [ axis, angle ] representation.
     * @param targetAxis A vector object to reuse for storing the axis.
     * @return An array, first element is the axis and the second is the angle in radians.
     */
    public Object[] toAxisAngle(Vec3 targetAxis) {
        this.normalize(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
        double angle = 2 * Math.acos(this.w);
        double s = Math.sqrt(1 - this.w * this.w); // assuming quaternion normalised then w is less than 1, so term always positive.
        if (s < 0.001) {
        	// test to avoid divide by zero, s is always positive due to sqrt
            // if s close to zero then direction of axis not important
            targetAxis.set(this.x, this.y, this.z);
        } else {
            targetAxis.set(this.x / s, this.y / s, this.z / s); //// normalise axis using (this.x)/s
        }
        return new Object[] { targetAxis, angle };
    }
    
    /**
     * Set the quaternion value given two vectors. The resulting rotation will be the needed rotation to rotate u to v.
     * @param u first vector to rotate from
     * @param v second vector to rotate to
     */
    public Quaternion setFromVectors( Vec3 u ,Vec3 v ) {
    	if (u.isAntiparallelTo(v,default_precision)) {
    	      Vec3 t1 = sfv_t1 ; //new Vec3();
    	      Vec3 t2 = sfv_t2 ;//new Vec3();
    	      
    	      u.tangents(t1, t2);
    	      this.setFromAxisAngle(t1, Math.PI);
    	}
    	else {
    		  Vec3 a = u.cross(v);
		      this.x = a.x ;
		      this.y = a.y ;
		      this.z = a.z ;
		      this.w = Math.sqrt(u.lengthSquared() * v.lengthSquared()) + u.dot(v) ;
		      this.normalize() ;
    	}
    		return this ;
    	}
    
    
    /**
     * default case of quaternion multiplication if no target Quaternion is given to store the result to 
     * @param quat
     * @return
     */
    public Quaternion mult(Quaternion quat) {
    	return mult(quat , new Quaternion());
    }
    
    /**
     * Multiply this quaternion with an other quaternion(quat) and stores result in targetQuat
     * @param quat
     * @param targetQuat Quaternion to hold result 
     * 
     */
    public Quaternion mult(Quaternion quat, Quaternion targetQuat) {
        double ax = this.x;
        double ay = this.y;
        double az = this.z;
        double aw = this.w;
        double bx = quat.x;
        double by = quat.y;
        double bz = quat.z;
        double bw = quat.w;
        targetQuat.x = ax * bw + aw * bx + ay * bz - az * by;
        targetQuat.y = ay * bw + aw * by + az * bx - ax * bz;
        targetQuat.z = az * bw + aw * bz + ax * by - ay * bx;
        targetQuat.w = aw * bw - ax * bx - ay * by - az * bz;
        return targetQuat;
    }
    
    /**
     * Default case of obtaining the inverse of a quaternion without specifying the target Quaternion 
     * in which the result could be stored so .......continued in return 
     * @return created the new target quaternion that stores the inversed Quat . 
     */
    public Quaternion inverse() {
    	return inverse(new Quaternion());
    }
    
    /**
     * inverse of q  = q_conjugate/(|q| squared) whereas |q|  = (w2 + x2 + y2 + z2) , w2is w squared 
     * Get the inverse quaternion rotation.
     */
    public Quaternion inverse(Quaternion targetQuat) {
        double x = this.x;
        double y = this.y;
        double z = this.z;
        double w = this.w;
        this.conjugate(targetQuat);
        double inorm2 = 1 / (x * x + y * y + z * z + w * w);
        targetQuat.x *= inorm2;
        targetQuat.y *= inorm2;
        targetQuat.z *= inorm2;
        targetQuat.w *= inorm2;
        return targetQuat;
    }
    
    /**
     * Default case similar to inverse .
     * @return conjugate of the Quaternion and storing it in the new quaternion passed in the return .
     */
    public Quaternion conjugate() {
    	return conjugate(new Quaternion());
    }
    /**
     * conjugate of q = (w + xi + yj + zk) is (w -xi -yj -zk) 
     * Get the quaternion conjugate
     */
    public Quaternion conjugate(Quaternion targetQuat) {
        targetQuat.x = -this.x;
        targetQuat.y = -this.y;
        targetQuat.z = -this.z;
        targetQuat.w = this.w;
        return targetQuat;
    }
    
    /**
     * Normalize the quaternion. Note that this changes the values of the quaternion.
     */
    public Quaternion normalize() {
        double l = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w);
        if (l == 0) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 0;
        } else {
            l = 1 / l;
            this.x *= l;
            this.y *= l;
            this.z *= l;
            this.w *= l;
        }
        return this;
    }
    
    /**
     * Approximation of quaternion normalization. Works best when quat is already almost-normalized.
     * @author unphased, https://github.com/unphased
     */
    public Quaternion normalizeFast() {
        double f = (3.0 - (this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w)) / 2.0;
        if (f == 0) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 0;
        } else {
            this.x *= f;
            this.y *= f;
            this.z *= f;
            this.w *= f;
        }
        return this;
    }
    
    /**
     * default case of no target vector given , so created one 
     * @param v
     * @return 
     */
    public Vec3 vmult(Vec3 v) {
    	return vmult(new Vec3());
    }

    /**
     * Multiply the quaternion by a vector
     */
    public Vec3 vmult(Vec3 v, Vec3 targetVec3) {
        double x = v.x;
        double y = v.y;
        double z = v.z;
        double qx = this.x;
        double qy = this.y;
        double qz = this.z;
        double qw = this.w;
        double ix = qw * x + qy * z - qz * y;
        double iy = qw * y + qz * x - qx * z;
        double iz = qw * z + qx * y - qy * x;
        double iw = -qx * x - qy * y - qz * z;
        double xComp = ix * qw + iw * -qx + iy * -qz - iz * -qy ;
        double yComp = iy * qw + iw * -qy + iz * -qx - ix * -qz ;
        double zComp = iz * qw + iw * -qz + ix * -qy - iy * -qx ;
        targetVec3.set(xComp , yComp ,zComp);
        return targetVec3;
    }
    
    /**
     * Copies value of source to this quaternion.
     * @return this
     */
    public Quaternion copy(Quaternion quat) {
        this.x = quat.x;
        this.y = quat.y;
        this.z = quat.z;
        this.w = quat.w;
        return this;
    } 
    
    /**
     * Convert the quaternion to euler angle representation. Order: YZX, as this page describes: https://www.euclideanspace.com/maths/standards/index.htm
     * @param order Three-character string, defaults to "YZX"
     */
    public void toEuler(Vec3 targetVec3, String order) {
        double heading = 0, attitude = 0, bank = 0;
        double x = this.x;
        double y = this.y;
        double z = this.z;
        double w = this.w;
        switch (order) {
            case "YZX":
                double test = x * y + z * w;
                if (test > 0.499) {
                    heading = 2 * Math.atan2(x, w);
                    attitude = Math.PI / 2;
                    bank = 0;
                }
                if (test < -0.499) {
                    heading = -2 * Math.atan2(x, w);
                    attitude = -Math.PI / 2;
                    bank = 0;
                }
                if (test != 0) {
                    double sqx = x * x;
                    double sqy = y * y;
                    double sqz = z * z;
                    heading = Math.atan2(2 * y * w - 2 * x * z, 1 - 2 * sqy - 2 * sqz);
                    attitude = Math.asin(2 * test);
                    bank = Math.atan2(2 * x * w - 2 * y * z, 1 - 2 * sqx - 2 * sqz);
                }
                break;
            default:
                throw new IllegalArgumentException("Euler order " + order + " not supported yet.");
        }
        targetVec3.set(bank ,heading, attitude);
        
    }
    
    /**
     * Set the quaternion components given Euler angle representation.
     *
     * @param order The order to apply angles: 'XYZ' or 'YXZ' or any other combination.
     *
     * See {@link https://www.mathworks.com/matlabcentral/fileexchange/20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors MathWorks} reference
     */
    public Quaternion setFromEuler(double x, double y, double z, String order) {
        double c1 = Math.cos(x / 2);
        double c2 = Math.cos(y / 2);
        double c3 = Math.cos(z / 2);
        double s1 = Math.sin(x / 2);
        double s2 = Math.sin(y / 2);
        double s3 = Math.sin(z / 2);
        if (order.equals("XYZ")) {
            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;
        } else if (order.equals("YXZ")) {
            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;
        } else if (order.equals("ZXY")) {
            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;
        } else if (order.equals("ZYX")) {
            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;
        } else if (order.equals("YZX")) {
            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;
        } else if (order.equals("XZY")) {
            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;
        }
        return this;
    }
    
    /**
     * cloning the Quaternion 
     */
    public Quaternion clone() {
        return new Quaternion(this.x, this.y, this.z, this.w);
    }

    /**
     * Performs a spherical linear interpolation between two quat
     *
     * @param toQuat second operand
     * @param t interpolation amount between the self quaternion and toQuat
     * @returns new Interpolated Quaternion
     */
    public Quaternion slerp(Quaternion toQuat, double t) {
    	return slerp(toQuat,t,new Quaternion());
    }
    /**
     * Performs a spherical linear interpolation between two quat
     *
     * @param toQuat second operand
     * @param t interpolation amount between the self quaternion and toQuat
     * @param targetQuat A quaternion to store the result in. If not provided, a new one will be created.
     * @returns {Quaternion} The "target" object
     */
    public Quaternion slerp(Quaternion toQuat, double t, Quaternion targetQuat) {
        double ax = this.x;
        double ay = this.y;
        double az = this.z;
        double aw = this.w;
        double bx = toQuat.x;
        double by = toQuat.y;
        double bz = toQuat.z;
        double bw = toQuat.w;
        double omega;
        double cosom;
        double sinom;
        double scale0;
        double scale1;
        
        // calc cosine
        cosom = ax * bx + ay * by + az * bz + aw * bw;
        
     // adjust signs (if necessary)
        if (cosom < 0.0) {
            cosom = -cosom;
            bx = -bx;
            by = -by;
            bz = -bz;
            bw = -bw;
        }
        
        // calculate coefficients
        if (1.0 - cosom > 0.000001) {
        	 // standard case (slerp)
            omega = Math.acos(cosom);
            sinom = Math.sin(omega);
            scale0 = Math.sin((1.0 - t) * omega) / sinom;
            scale1 = Math.sin(t * omega) / sinom;
        }
        else {
        	// "from" and "to" quaternions are very close
            //  ... so we can do a linear interpolation
            scale0 = 1.0 - t;
            scale1 = t;
        }
        
        // calculate final values
        targetQuat.x = scale0 * ax + scale1 * bx;
        targetQuat.y = scale0 * ay + scale1 * by;
        targetQuat.z = scale0 * az + scale1 * bz;
        targetQuat.w = scale0 * aw + scale1 * bw;

        return targetQuat;
    }
    
    public Quaternion integrate(Vec3 angularVelocity, double dt, Vec3 angularFactor) {
    	return  integrate( angularVelocity,  dt,  angularFactor , new Quaternion()) ;
    }
    
    /**
     * Rotate an absolute orientation quaternion given an angular velocity and a time step.
     */
    public Quaternion integrate(Vec3 angularVelocity, double dt, Vec3 angularFactor, Quaternion targetQuat) {
        double ax = angularVelocity.x * angularFactor.x;
        double ay = angularVelocity.y * angularFactor.y;
        double az = angularVelocity.z * angularFactor.z;
        double bx = this.x;
        double by = this.y;
        double bz = this.z;
        double bw = this.w;
        
        double half_dt = dt * 0.5;
        
        targetQuat.x += half_dt * (ax * bw + ay * bz - az * by);
        targetQuat.y += half_dt * (ay * bw + az * bx - ax * bz);
        targetQuat.z += half_dt * (az * bw + ax * by - ay * bx);
        targetQuat.w += half_dt * (-ax * bx - ay * by - az * bz);
        return targetQuat;
    }

    static Vec3 sfv_t1 = new Vec3() ; 
    static Vec3 sfv_t2 = new Vec3() ;
}
	
    
    
    
    

