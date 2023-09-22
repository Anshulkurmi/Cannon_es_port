package material;

public class ContactMaterialOptions {
    /**
       * Friction coefficient.
       * @default 0.3
    */
    protected double friction ;
    /**
       * Restitution coefficient.
       * @default 0.3
       */
    protected double restitution ;
    /**
       * Stiffness of the produced contact equations.
       * @default 1e7
       */
    protected double contactEquationStiffness ;
    /**
       * Relaxation time of the produced contact equations.
       * @default 3
       */
    protected double contactEquationRelaxation ;
    /**
       * Stiffness of the produced friction equations.
       * @default 1e7
       */
    protected double frictionEquationStiffness ;
    /**
       * Relaxation time of the produced friction equations
       * @default 3
       */
    protected double frictionEquationRelaxation ;

    public ContactMaterialOptions() {
        this.friction = 0.3;
        this.restitution = 0.3;
        this.contactEquationStiffness =1e7;
        this.contactEquationStiffness = 3;
        this.frictionEquationRelaxation = 1e7;
        this.frictionEquationStiffness = 3;
    }

    public ContactMaterialOptions(double friction , double restitution , double contactEquationStiffness ,double contactEquationRelaxation,double frictionEquationStiffness ,double frictionEquationRelaxation){
        this.friction = friction;
        this.restitution = restitution;
        this.contactEquationStiffness = contactEquationStiffness ;
        this.contactEquationRelaxation = contactEquationRelaxation;
        this.frictionEquationStiffness = frictionEquationStiffness;
        this.frictionEquationRelaxation = frictionEquationRelaxation ;
         
    }

}
