package events;

import objects.Body;

public class ContactEvent extends Event {
        public Body bodyA;
        public Body bodyB;

        public ContactEvent(String type) {
        	super(type,null);
            this.type = type;
            this.bodyA = null;
            this.bodyB = null;
        }
    }