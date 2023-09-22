package math;

/**
 * An element containing 6 entries, 3 spatial and 3 rotational degrees of freedom.
 */

public class JacobianElement {
	/**
	 * spatial
	 */
	public Vec3 spatial ;
	
	/**
	 * rotational
	 */
	public Vec3 rotational ;
	
	public JacobianElement(){
		spatial = new Vec3();
		rotational = new Vec3();
	}
	
	/**
	   * Multiply with other JacobianElement
	   */
	public double multiplyElement( JacobianElement element ){
	    return element.spatial.dot(this.spatial) + element.rotational.dot(this.rotational) ;
	}
	
	/**
	   * Multiply with two vectors
	   */
	public double multiplyVectors(Vec3 spatial,Vec3 rotational){
	    return spatial.dot(this.spatial) + rotational.dot(this.rotational) ;
	}
}
