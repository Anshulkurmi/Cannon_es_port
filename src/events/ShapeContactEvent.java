package events;

import objects.Body;
import shapes.Shape;

public class ShapeContactEvent extends Event{
        public Body bodyA;
        public Body bodyB;
        public Shape shapeA;
        public Shape shapeB;

        public ShapeContactEvent(String type) {
        	super(type,null);
            this.type = type;
            this.bodyA = null;
            this.bodyB = null;
            this.shapeA = null;
            this.shapeB = null;
        }
    }