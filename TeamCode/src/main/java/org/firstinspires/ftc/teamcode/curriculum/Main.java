package org.firstinspires.ftc.teamcode.curriculum;

public class Main {
    public void run() {

        Robot friday = new Robot();
        friday.introduceMyself();
        friday.sayAge();
        friday.veryComplexAction();

        Animal generalAnimal = new Animal();
        generalAnimal.eat();

        Reptile generalReptile = new Reptile();
        generalReptile.eat();
        generalReptile.layEgg();

        Turtle turtle = new Turtle();
        turtle.limbs = 4;
        turtle.lifespan = 300;
        turtle.hasShell = true;

        Snake snake1 = new Snake();
        Snake jeremiah = new Snake();
        Snake bob = new Snake();

        snake1.limbs = 0;
        jeremiah.limbs = 0;
        bob.limbs = 0;

        snake1.species = "Anaconda";
        snake1.eat();
        snake1.slither();

        jeremiah.species = "Python";
        jeremiah.layEgg();
        jeremiah.slither();

        bob.species = "Python";
        bob.eat();

        Direction bruh = Direction.FORWARD;
        Direction bruh2 = Direction.REVERSE;

        RunMode bruh3 = RunMode.RUN_TO_POSITION;
        RunMode bruh4 = RunMode.RUN_USING_ENCODER;
        RunMode bruh5 = RunMode.RUN_WITHOUT_ENCODER;
        RunMode bruh6 = RunMode.STOP_AND_RESET_ENCODER;

    }
}

class Animal {
    public int lifespan;
    public String species;
    public int limbs;

    public void eat() {
        System.out.println("I am eating!");
    }
}

class Reptile extends Animal{
    public void layEgg() {
        System.out.println("I am laying eggs!");
    }
}

class Turtle extends Reptile {
    public boolean hasShell;
}

class Snake extends Reptile {
    public void slither() {
        System.out.println("I am slithering!");
    }
}
