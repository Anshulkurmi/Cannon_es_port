package objects;

import math.Vec3;
public class WheelOptions {
	/** The wheel body */
	public Body body ;
	
    /** Position of the wheel, locally in the chassis body. */
	public Vec3 position ;
	
    /** Axis of rotation of the wheel, locally defined in the chassis. */
	public Vec3 axis ;
	
    /** Slide direction of the wheel along the suspension. */
	public Vec3 direction ;
	
	public WheelOptions() {
		this(new Body() , new Vec3(), new Vec3() , new Vec3());
	}
	
	public WheelOptions(Body body,Vec3 position,Vec3 axis,Vec3 direction) {
		this.body = body ;
		this.position = position ;
		this.axis = axis ;
		this.direction = direction ;
	}
	
}
