package objects;

import math.Vec3;
public class RigidVehicleOptions {
	/**
     * A Vector3 defining the world coordinate system.
     * @default new Vec3(1, 2, 3)
     */
	public Vec3 coordinateSystem ;
	/**
     * Optionally pass a body for the chassis
     */
	public Body chassisBody ;
	
	public RigidVehicleOptions() {
		 this(new Vec3(1.0,2.0,3.0) , new Body());
	}

	public RigidVehicleOptions(Vec3 coordinateSystem , Body chassisBody){
		this.coordinateSystem = coordinateSystem ;
		this.chassisBody = chassisBody ;		
	}
	
}
