package examples;

import math.Vec3;
import objects.Body;
import objects.BodyOptions;
import shapes.Box;
import shapes.Particle;
import shapes.Plane;
import shapes.Sphere;
import world.World;
import world.WorldOptions;

public class Ball {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		World world = new World(new WorldOptions());

        // Create a ground plane
        Body ground = new Body();
        ground.addShape(new Plane());
        world.addBody(ground);

        // Create a box to fall under gravity
        Body box = new Body(new BodyOptions(1,new Box(new Vec3(1, 1, 1))));
        box.position = new Vec3(0, 10 , 0); // Initial position
        //box.mass(1); // Set mass
        world.addBody(box);
        
        //for(int i = 0 ; i < 10 ; i++) {
        	Body sphere = new Body(new BodyOptions(1,new Sphere(1)));
            sphere.position = new Vec3(2,5,0) ;
            sphere.velocity = new Vec3(1,0,0);
            
            world.addBody(sphere);
        //}
        
        for(Body body : world.bodies) {
        	System.out.println(body.position);
        }
        
        BodyOptions particleOptions = new BodyOptions() ;
        particleOptions.setShape(new Particle()) ;
        particleOptions.setMass(1);
        Body particle = new Body(particleOptions);
        particle.position  = new Vec3() ;
        // System.out.println(world.getBodyById(8));
        
        // Simulation loop
        for (int i = 0; i < 100; i++) {
            // Simulate physics for a time step
            world.step(1.0 , 0 , 10); // Assume 60 FPS

            // Output box position (for demonstration)
            System.out.println("Time Step " + i + ": sphere Position = " + sphere.position + "\n "   );
        }
        
        

	}

}
