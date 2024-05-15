package events;

import equations.ContactEquation;
import objects.Body;

public class CollideEvent extends Event{
	  public Body body;
      public ContactEquation contact;
      
      public CollideEvent(String type) {
      	  super(type,null);
          this.body = null;
          this.contact=null;
      }
}
