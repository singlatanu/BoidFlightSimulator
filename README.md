# Boid Flight Simulator

**This project implements the swarming behavior observed in flocks of starling birds and schools of fish using [Boids model](https://en.wikipedia.org/wiki/Boids).**

Swarming behavior emerges from a small set of simple local interaction rules followed by each individual (boid). The three primary rules governing the motion of boids are::
- **Alignment**: Boids tend to align their velocity with the average velocity of nearby boids.

- **Cohesion**: Boids tend to move toward the average position (center of mass) of their neighbors.

- **Separation**: Boids maintain a minimum distance from nearby boids to avoid collisions.

In addition to these core rules, other effects such as random motion, environmental drag, or noise can be included. However, the three rules above have the highest priority and are sufficient to produce realistic collective motion.

Two important factors determine how boids interact with their neighbors:

- Radius of Vision: Defines how far a boid can see and detect other boids.

- Field of Vision: Defines the angular range within which a boid can perceive its neighbors.

---
# Running the script: 
The parameters are defined at the end of the script. Set them to the desired values and run the script with Python.
