package shapes;

import math.Vec3; 
import math.Quaternion; 
import objects.Body; 
import material.Material; 

/**
 * Abstract base class for shapes.
 */
public abstract class Shape {
	/**
	 * Identifier of the Shape.
	 */
	public int id;
	
	/**
	 * The type of this shape. Must be set to a valid ShapeTypes value.
	 */
	public ShapeTypes type;

	/**
	 * Static counter to assign unique IDs to shapes.
	 */
	private static int idCounter = 0;

	
	/**
	 * The local bounding sphere radius of this shape.
	 */
	public double boundingSphereRadius;

	/**
	 * Whether to produce contact forces when in contact with other bodies. Note
	 * that contacts will be generated, but they will be disabled. Default is true.
	 */
	public boolean collisionResponse;

	/**
	 * Collision filter group for this shape. Default is 1.
	 */
	public int collisionFilterGroup;

	/**
	 * Collision filter mask for this shape. Default is -1.
	 */
	public int collisionFilterMask;

	/**
	 * Optional material of the shape that regulates contact properties.
	 */
	public Material material;

	/**
	 * The body to which the shape is added to.
	 */
	public Body body;


	/**
	 * All the available Shape types.
	 */
	public static final ShapeTypes[] types = ShapeTypes.values();

	public Shape(ShapeTypes type) {
		this(type, null);
	}

	/**
	 * Constructor for Shape.
	 *
	 * @param options Configuration options for the shape.
	 */
	public Shape(ShapeTypes type, ShapeOptions options) {
		this.id = idCounter++;
		this.type = type;
		if (options != null) {
			this.boundingSphereRadius = 0;
			this.collisionResponse = options.collisionResponse;
			this.collisionFilterGroup = options.collisionFilterGroup;
			this.collisionFilterMask = options.collisionFilterMask;
			this.material = options.material;
		}
		else {
			this.boundingSphereRadius = 0;
			this.collisionResponse = true ;
			this.collisionFilterGroup = 1;
	    	this.collisionFilterMask = -1;
	    	this.material = null ;
		}
		this.body = null;
	}

	/**
	 * Computes the bounding sphere radius. The result is stored in the property
	 * .boundingSphereRadius.
	 */
	public abstract void updateBoundingSphereRadius();

	/**
	 * Get the volume of this shape.
	 */
	public abstract double volume();

	  /**
   * Calculates the inertia in the local frame for this shape.
   * @see http://en.wikipedia.org/wiki/List_of_moments_of_inertia
   */
	public abstract Vec3 calculateLocalInertia(double mass, Vec3 target);
	//changed Vec3 to void 
	/**
	 * Calculates the world-space axis-aligned bounding box for this shape.
	 *
	 * @param pos  World position of the shape.
	 * @param quat World rotation quaternion of the shape.
	 * @param min  Output vector to store the minimum corner of the AABB.
	 * @param max  Output vector to store the maximum corner of the AABB.
	 */
	public abstract void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max);

	
}
