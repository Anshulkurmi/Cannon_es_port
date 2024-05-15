package constraints;

import objects.Body;

public class MotorHingeConstraint extends HingeConstraint{
	
	public double motorTargetVelocity ;
	
	public MotorHingeConstraint(Body bodyA, Body bodyB , HingeConstraintOptions options) {
		super(bodyA, bodyB , options);
	// TODO Auto-generated constructor stub
		
	}

}
