package constraints;

public class ConstraintOptions {
	/**
	 * Set to false if you don't want the bodies to collide when they are connected.
	 * 
	 * @default true
	 */
	boolean collideConnected;
	/**
	 * Set to false if you don't want the bodies to wake up when they are connected.
	 * 
	 * @default true
	 */
	boolean wakeUpBodies;

	public ConstraintOptions(boolean collideConnected, boolean wakeUpBodies) {
		this.collideConnected = collideConnected;
		this.wakeUpBodies = wakeUpBodies;
	}
}
