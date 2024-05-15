package objects;

import java.util.ArrayList;
import java.util.List;

import constraints.HingeConstraint;
import constraints.HingeConstraintOptions;
import constraints.MotorHingeConstraint;
import events.Event;
import math.Vec3;
import shapes.Box;
import shapes.Sphere;
import world.World;


public class RigidVehicle {
    private List<Body> wheelBodies; // List to store wheel bodies
    private Vec3 coordinateSystem; // Coordinate system for the vehicle
    private Body chassisBody; // Chassis body of the vehicle
    private List<MotorHingeConstraint> constraints; // Constraints for the wheels
    private List<Vec3> wheelAxes; // Axes of rotation for the wheels
    private List<Double> wheelForces; // Forces applied to the wheels

    public RigidVehicle(RigidVehicleOptions options) {
        this.wheelBodies = new ArrayList<>(); // Initialize wheel bodies list
        this.coordinateSystem = options.coordinateSystem != null ? options.coordinateSystem.clone() : new Vec3(1, 2, 3); // Set the coordinate system for the vehicle

        if (options.chassisBody != null) {
            this.chassisBody = options.chassisBody;
        } else {
            // No chassis body given. Create it!
            this.chassisBody = new Body(new BodyOptions(1, new Box(new Vec3(5, 0.5, 2)))); // Create a default chassis body
        }

        this.constraints = new ArrayList<>(); // Initialize constraints list
        this.wheelAxes = new ArrayList<>(); // Initialize wheel axes list
        this.wheelForces = new ArrayList<>(); // Initialize wheel forces list
    }

    // Method to add a wheel to the vehicle
    public int addWheel(WheelOptions options) {
        Body wheelBody;

        if (options.body != null) {
            wheelBody = options.body;
        } else {
            // No wheel body given. Create it!
            wheelBody = new Body(new BodyOptions(1, new Sphere(1.2))); // Create a default wheel body
        }

        this.wheelBodies.add(wheelBody); // Add the wheel body to the list
        this.wheelForces.add(0.0); // Initialize wheel force

        // Position constrain wheels
        Vec3 position = options.position != null ? options.position.clone() : new Vec3();
        Vec3 worldPosition = new Vec3();
        this.chassisBody.pointToWorldFrame(position, worldPosition);
        wheelBody.position.set(worldPosition.x, worldPosition.y, worldPosition.z);

        // Constrain wheel
        Vec3 axis = options.axis != null ? options.axis.clone() : new Vec3(0, 0, 1);
        this.wheelAxes.add(axis);

        MotorHingeConstraint hingeConstraint = new MotorHingeConstraint(this.chassisBody, wheelBody, new HingeConstraintOptions(position,axis,Vec3.ZERO,axis,false));
        this.constraints.add(hingeConstraint); // Add the hinge constraint

        return this.wheelBodies.size() - 1;
    }

    // Method to set the steering value of a wheel
    public void setSteeringValue(double value, int wheelIndex) {
        Vec3 axis = this.wheelAxes.get(wheelIndex);

        double c = Math.cos(value);
        double s = Math.sin(value);
        double x = axis.x;
        double z = axis.z;

        // Set the angle of the hinge axis
        this.constraints.get(wheelIndex).axisA.set(-c * x + s * z, 0.0, s * x + c * z);
    }

    // Method to set the target rotational speed of the hinge constraint
    public void setMotorSpeed(double value, int wheelIndex) {
        MotorHingeConstraint hingeConstraint = this.constraints.get(wheelIndex);
        hingeConstraint.enableMotor();
        hingeConstraint.motorTargetVelocity = value;
    }

    // Method to disable the motor of the hinge constraint
    public void disableMotor(int wheelIndex) {
        HingeConstraint hingeConstraint = this.constraints.get(wheelIndex);
        hingeConstraint.disableMotor();
    }

    // Method to set the wheel force to apply on one of the wheels each time step
    public void setWheelForce(double value, int wheelIndex) {
        this.wheelForces.set(wheelIndex, value);
    }

    // Method to apply a torque on one of the wheels
    public void applyWheelForce(double value, int wheelIndex) {
        Vec3 axis = this.wheelAxes.get(wheelIndex);
        Body wheelBody = this.wheelBodies.get(wheelIndex);
        Vec3 bodyTorque = wheelBody.torque;

        Vec3 torque = new Vec3();
        axis.scale(value, torque);
        wheelBody.vectorToWorldFrame(torque, torque);
        bodyTorque.vadd(torque, bodyTorque);
    }

    // Method to add the vehicle including its constraints to the world
    public void addToWorld(World world) {
    	//HingeConstraint contraints = this.constraints ;
    	
        List<Body> bodies = new ArrayList<>(this.wheelBodies);
        bodies.add(this.chassisBody);

        for (Body body : bodies) {
            world.addBody(body);
        }

        for (HingeConstraint constraint : this.constraints) {
            world.addConstraint(constraint);
        }

        world.addEventListener("preStep", this::_update);
    }

    private void _update(Event event) {
        for (int i = 0; i < this.wheelForces.size(); i++) {
            this.applyWheelForce(this.wheelForces.get(i), i);
        }
    }

    // Method to remove the vehicle including its constraints from the world
    public void removeFromWorld(World world) {
        List<Body> bodies = new ArrayList<>(this.wheelBodies);
        bodies.add(this.chassisBody);

        for (Body body : bodies) {
            world.removeBody(body);
        }

        for (HingeConstraint constraint : this.constraints) {
            world.removeConstraint(constraint);
        }
    }

    // Method to get the current rotational velocity of a wheel
    public double getWheelSpeed(int wheelIndex) {
        Vec3 axis = this.wheelAxes.get(wheelIndex);
        Body wheelBody = this.wheelBodies.get(wheelIndex);
        Vec3 w = wheelBody.angularVelocity;
        Vec3 worldAxis = new Vec3();
        this.chassisBody.vectorToWorldFrame(axis, worldAxis);
        return w.dot(worldAxis);
    }
}
