package math;

import java.util.Random;

public class Vec3 {
    public double x, y, z;

    //added
    public static Vec3 ZERO = new Vec3(0,0,0);
    public static Vec3 UNIT_X = new Vec3(1,0,0);
    public static Vec3 UNIT_Y = new Vec3(0,1,0);
    public static Vec3 UNIT_Z = new Vec3(0,0,1);

    public static double default_precision=1e-6;

    /*default constructor  */
    public Vec3() {
    	this.x = 0;
        this.y = 0;
        this.z = 0;
    }
    
    //public constructor
    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

   /*
    * Cross product of two vectors and new Vec3() to save its result
    */
    public Vec3 cross(Vec3 v) {
    	return cross(v, new Vec3());   
    }
    
     /**
   * Vector cross product with Vector v
   * @param target Optional target to save in.
   */
    public Vec3 cross(Vec3 v , Vec3 target ) {
    	target.x = this.y * v.z - this.z * v.y ;
    	target.y = this.z * v.x - this.x * v.z ;
    	target.z  = this.x * v.y - this.y * v.x ;
    	return target ;
    }

    /**
   * Set the vectors' 3 elements
   */
    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
   * Set all components of the vector to zero.
   */
    public void setZero() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    public Vec3 vadd(Vec3 v) {
        return new Vec3(
            this.x + v.x,
            this.y + v.y,
            this.z + v.z
        );
    }
    
    /**
   * Vector addition
   * @param v : Vector to add
   * @param target : saving added output vector 
   * @return : target vector
   */
  //changed return type Void to Vec3 
    public Vec3 vadd(Vec3 v, Vec3 target ) {
    	target.x =  this.x + v.x ;
    	target.y = 	this.y + v.y ;
    	target.z = 	this.z + v.z ;
        return target ;
    }


    public Vec3 vsub(Vec3 v) {
        return new Vec3(
            this.x - v.x,
            this.y - v.y,
            this.z - v.z
        );
    }
    
    /**
   * Vector subtraction
   * @param v : vector to subtract
   * @param target Optional target to save in.
   */
    public void vsub(Vec3 v, Vec3 target ) {
    	target.x =  this.x - v.x ;
    	target.y = 	this.y - v.y ;
    	target.z = 	this.z - v.z ;
    }

    /**
   * Get the cross product matrix a_cross from a vector, such that a x b = a_cross * b = c
   *
   * See {@link https://www8.cs.umu.se/kurser/TDBD24/VT06/lectures/Lecture6.pdf Umeå University Lecture}
   */
   //added method crossmat , was missing initially 
    public Mat3 crossmat(){
        return new Mat3(new double[]{0, -this.z, this.y, this.z, 0, -this.x, -this.y, this.x, 0}) ;
    }
    
    /**
   * Normalize the vector. Note that this changes the values in the vector.
   * @return Returns the norm of the vector
   */
    public double normalize() {
        double length = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
        if (length > 0) {
            double invLength = 1.0 / length;
            this.x *= invLength;
            this.y *= invLength;
            this.z *= invLength;
            return length;
        } else {
            // Cannot normalize a zero vector
            return -1;
        }
    }
    
   //default unit() method without target vector 
    public Vec3 unit() {
    	return unit(new Vec3());
    }
     /**
   * Get the version of this vector that is of length 1.
   * @param target Optional target to save in
   * @return Returns the unit vector
   */
    public Vec3 unit(Vec3 target) {
        target = new Vec3(this.x, this.y, this.z);
        target.normalize();
        return target;
    }

    /**
   * Get the length of the vector
   */
    public double length() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    /**
   * Get the squared length of the vector.
   */
    public double lengthSquared() {
        return this.x * this.x + this.y * this.y + this.z * this.z;
    }

    /**
   * Get distance from this point to another point
   */
    public double distanceTo(Vec3 v) {
        double dx = this.x - v.x;
        double dy = this.y - v.y;
        double dz = this.z - v.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
   * Get squared distance from this point to another point
   */
    public double distanceSquared(Vec3 v) {
        double dx = this.x - v.x;
        double dy = this.y - v.y;
        double dz = this.z - v.z;
        return dx * dx + dy * dy + dz * dz;
    }
    
    //default method scale without target Vector 
    public Vec3 scale(double scalar) {
    	return scale(scalar , new Vec3() );
    }
    /**
   * Multiply all the components of the vector with a scalar.
   * @param target The vector to save the result in.
   */
    public Vec3 scale(double scalar , Vec3 target) {
        
            target.x = this.x * scalar ;
            target.y = this.y * scalar ;
            target.z = this.z * scalar ;
        
            return target ;
    }
    

    public Vec3 vmul(Vec3 v) {
    	return vmul(v , new Vec3());
    }
    
    /**
     * Multiply corresponding x,y,z components with vector v and store that result in target 
     * @param v 
     * @return target 
     */
    public Vec3 vmul(Vec3 v, Vec3 target) {
       
        target.x = this.x * v.x ;
        target.y = this.y * v.y ;
        target.z = this.z * v.z ;
        
        return target ;
    }
    

    public Vec3 addScaledVector(double scalar, Vec3 v) {
    	return addScaledVector( scalar,  v , new Vec3()) ;
    }
    
    /**
   * Scale a vector and add it to this vector. Save the result in "target". (target = this + vector * scalar)
   * @param target The vector to save the result in.
   */
    public Vec3 addScaledVector(double scalar, Vec3 v, Vec3 target) {
        
         target.x =   this.x + scalar * v.x ; 
         target.y =   this.y + scalar * v.y ; 
         target.z =   this.z + scalar * v.z ;
         
         return target;
    }

    /**
   * Calculate dot product
   * @param vector
   */
    public double dot(Vec3 v) {
        return this.x * v.x + this.y * v.y + this.z * v.z;
    }

    public boolean isZero() {
        return this.x == 0 && this.y == 0 && this.z == 0;
    }

    
    public Vec3 negate() {
    	return negate(new Vec3());
    }
    /**
   * Make the vector point in the opposite direction.
   * @param target Optional target to save in
   */
    public Vec3 negate(Vec3 target) {
    	target.x = -this.x ;
    	target.y = -this.y ;
    	target.z = -this.z ;
        return target ;
    }

    /**
   * Compute two artificial tangents to the vector
   * @param t1 Vector object to save the first tangent in
   * @param t2 Vector object to save the second tangent in
   */
    public void tangents(Vec3 t1, Vec3 t2) {
        double norm = this.normalize();
        if (norm == -1) {
            // Cannot compute tangents for a zero vector
            return;
        }
        Random rand = new Random();
        Vec3 randVec = new Vec3(rand.nextDouble(), rand.nextDouble(), rand.nextDouble());
        t1.copy(randVec.cross(this));
        t2.copy(this.cross(t1));
    }

    /**
   * Converts to a more readable format
   */
    @Override
    public String toString() {
        return this.x + "," + this.y + "," + this.z;
    }

    /**
   * Converts to an array
   */
    public double[] toArray() {
        return new double[]{this.x, this.y, this.z};
    }

    /**
   * Copies value of source to this vector.
   */
    public void copy(Vec3 v) {
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
    }
    
    public void lerp(Vec3 v, double alpha) {
    	lerp (v , alpha , new Vec3());
    }
    /**
   * Do a linear interpolation between two vectors
   * @param alpha:  A number between 0 and 1. 0 will make this function return u, and 1 will make it return v. Numbers in between will generate a vector in between them.
   */
    public void lerp(Vec3 v, double alpha, Vec3 target) {
        double x = this.x;
        double y = this.y;
        double z = this.z;
        target.x = x + (v.x - x) * alpha;
        target.y = y + (v.y - y) * alpha;
        target.z = z + (v.z - z) * alpha;
    }

   
    public boolean almostEquals(Vec3 v) {
    	return almostEquals(v,default_precision);
    }
     /**
   * Check if a vector equals is almost equal to another one.
   */
    public boolean almostEquals(Vec3 v, double precision) {
        return (
            Math.abs(this.x - v.x) < precision &&
            Math.abs(this.y - v.y) < precision &&
            Math.abs(this.z - v.z) < precision
        );
    }

    
    public boolean almostZero() {
    	return almostZero(default_precision);
    }
    /**
   * Check if a vector is almost zero
   */
    public boolean almostZero(double precision) {
        return (
            Math.abs(this.x) < precision &&
            Math.abs(this.y) < precision &&
            Math.abs(this.z) < precision
        );
    }
    
    public boolean isAntiparallelTo(Vec3 v) {
    	return isAntiparallelTo(v, default_precision);
    }
    /**
   * Check if the vector is anti-parallel to another vector.
   * @param precision Set to zero for exact comparisons
   */
    public boolean isAntiparallelTo(Vec3 v, double precision) {
        return this.dot(v) < -1 + precision;
    }

    /**
   * Clone the vector
   */
    public Vec3 clone() {
        return new Vec3(this.x, this.y, this.z);
    }
}
