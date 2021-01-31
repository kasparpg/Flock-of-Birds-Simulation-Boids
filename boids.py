import math
from random import randint, uniform

import pygame as pg

# Color constants
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# Window Parameters
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

# Parameters
NUM_BOIDS = 200
BOID_SIZE = 10
SPEED = 3.5
MAX_FORCE = 0.3
BOID_FRICTION = 0.75

WANDER_RADIUS = 30

SEPARATION = 1.2
SEPARATION_RADIUS = 40

ALIGNMENT = 1
ALIGNMENT_RADIUS = 50

COHESION = 1
COHESION_RADIUS = 80


class Simulation:

    def __init__(self):
        pg.init()
        self.running = False
        self.clock = pg.time.Clock()
        self.screen = pg.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.screen_rect = self.screen.get_rect()
        self.fps = 60

        self.boids = []
        for i in range(NUM_BOIDS):
            self.boids.append(Boid(self, (randint(0, SCREEN_WIDTH), randint(0, SCREEN_HEIGHT))))

    def events(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.running = False

    def draw(self):
        # Empty the last screen
        self.screen.fill(BLACK)

        # Draw all boids
        for boid in self.boids:
            boid.draw(self.screen)

        # Update the screen
        pg.display.update()

    def update(self):
        """
        Method for going one step in the simulation
        """
        for boid in self.boids:
            boid.update()

    def run(self):
        """
        Runs the simulation
        """
        self.running = True
        while self.running:
            self.clock.tick(self.fps)
            self.events()
            self.update()
            self.draw()

class PhysicsObject:

    def __init__(self, simulation, position):
        self.simulation = simulation
        self.acc = pg.math.Vector2(0, 0)
        self.vel = pg.math.Vector2(0, 0)
        self.pos = pg.math.Vector2(position)

        self.speed = 1

        self.friction = 0.9
    def getVel(self):
        return self.vel

    def update(self):
        self.vel += self.acc
        self.pos += self.vel * self.speed

        # Reset acceleration
        self.acc *= 0

        # Simplistic surface friction
        self.vel *= self.friction

        # wrap around the edges of the screen
        if self.pos.x > self.simulation.screen_rect.w:
            self.pos.x -= self.simulation.screen_rect.w
        elif self.pos.x < 0:
            self.pos.x += self.simulation.screen_rect.w

        if self.pos.y > self.simulation.screen_rect.h:
            self.pos.y -= self.simulation.screen_rect.h
        elif self.pos.y < 0:
            self.pos.y += self.simulation.screen_rect.h


class Boid(PhysicsObject):

    def __init__(self, simulation, position):
        super().__init__(simulation, position)
        self.speed = SPEED  # Max speed
        self.vel = pg.math.Vector2(randint(-2, 2), randint(-2, 2))  # Random initial velocity

        self.max_force = MAX_FORCE  # force cap, limits the size of the different forces
        self.friction = BOID_FRICTION  # Friction coefficient for the simplistic physics

        # Parameters for wandering behaviour
        self.target = pg.math.Vector2(0, 0)
        self.future_loc = pg.math.Vector2(0, 0)
        self.theta = uniform(-math.pi, math.pi)

    def update(self):
        """
        Updates the acceleration of the boid by adding together the different forces that acts on it
        """
        self.acc += self.wander()  # Wandering force
        self.acc += self.separation() * SEPARATION  # separation force scaled with a controll parameter
        self.acc += self.alignment() * ALIGNMENT  # alignement force scaled with a controll parameter
        self.acc += self.cohesion() * COHESION  # cohesion force scaled with a controll parameter

        # move by calling super
        super().update()

    def vision(self):
        return math.atan2(self.vel.y, self.vel.x)

    def separation(self):
        """
        Calculate the separation force vector
        Separation: steer to avoid crowding local flockmates
        :return force vector
        """

        force_vector = pg.math.Vector2(0, 0)
        for boid in self.simulation.boids:
            if boid == self:
                continue
            x_diff = boid.pos.x - self.pos.x
            y_diff = boid.pos.y - self.pos.y
            distance = math.sqrt((x_diff)**2+(y_diff)**2)

            if(distance < SEPARATION_RADIUS):
                deg = math.atan2(y_diff, x_diff)
                if(-160 <((self.vision()*180/math.pi)-(deg*180/math.pi))< 160):
                    force_vector = pg.math.Vector2(-math.cos(deg), -math.sin(deg))
        return force_vector

    def alignment(self):
        """
        Calculate the alignment force vector
        Alignment: steer towards the average heading of local flockmates
        :return force vector
        """
        force_vector = pg.math.Vector2(0, 0)
        aligment_boids_x = []
        aligment_boids_y = []
        for boid in self.simulation.boids:
            if boid == self:
                continue
            x_diff = boid.pos.x - self.pos.x
            y_diff = boid.pos.y - self.pos.y
            distance = math.sqrt((x_diff) ** 2 + (y_diff) ** 2)

            if (distance < ALIGNMENT_RADIUS):
                deg = math.atan2(y_diff, x_diff)
                if(-160 <((self.vision()*180/math.pi)-(deg*180/math.pi))< 160):
                    aligment_boids_x.append(boid.vel.x)
                    aligment_boids_y.append(boid.vel.y)
        if len(aligment_boids_x) > 0:
           average_x = sum(aligment_boids_x) / len(aligment_boids_x)
           average_y = sum(aligment_boids_y) / len(aligment_boids_y)
           deg2 = math.atan2(average_x - self.pos.x, average_y - self.pos.y)
           force_vector = pg.math.Vector2(math.cos(deg2), math.sin(deg2))
        return force_vector

    def cohesion(self):
        """
        Calculate the cohesion force vector
        Cohesion: steer to move toward the average position of local flockmates
        """
        force_vector = pg.math.Vector2(0, 0)
        cohesion_boids_x = []
        cohesion_boids_y = []
        for boid in self.simulation.boids:
            if boid == self:
                continue
            x_diff = boid.pos.x - self.pos.x
            y_diff = boid.pos.y - self.pos.y
            distance = math.sqrt((x_diff) ** 2 + (y_diff) ** 2)
            if (distance < COHESION_RADIUS):
                deg = math.atan2(y_diff, x_diff)
                if(-160 <((self.vision()*180/math.pi)-(deg*180/math.pi))< 160):
                    cohesion_boids_x.append(boid.pos.x)
                    cohesion_boids_y.append(boid.pos.y)
        if len(cohesion_boids_x) > 0:
            center_x = sum(cohesion_boids_x)/len(cohesion_boids_x)
            center_y = sum(cohesion_boids_y)/len(cohesion_boids_y)
            deg2 = math.atan2(center_x-self.pos.x, center_y-self.pos.y)
            force_vector = pg.math.Vector2(math.cos(deg2), math.sin(deg2))

        return force_vector

    def move_towards_target(self, target):
        """
        Calculate force vector for moving the boid to the target
        """
        # vector to the target
        desired = target - self.pos

        distance = desired.length()
        desired = desired.normalize()

        # Radius
        radius = 100

        if distance < radius:
            # if the distance is less than the radius,
            m = remap(distance, 0, radius, 0, self.speed)

            # scale the desired vector up to continue movement in that direction
            desired *= m
        else:
            desired *= self.speed

        force_vector = desired - self.vel
        limit(force_vector, self.max_force)
        return force_vector

    def wander(self):
        """
        Calcualte a random target to move towards to get natural random flight
        """
        if self.vel.length_squared() != 0:
            # Calculate where you will be in the future
            self.future_loc = self.vel.normalize() * 80

            # Calculate a random angle addition
            self.theta += uniform(-math.pi, math.pi) / 10

            # set the target to your position + your future position + a distance in the direction of the random angle
            self.target = self.pos + self.future_loc + pg.math.Vector2(WANDER_RADIUS * math.cos(self.theta),
                                                                       WANDER_RADIUS * math.sin(self.theta))
        return self.move_towards_target(self.target)

    def draw(self, screen):
        """Draw boid to screen"""

        # Calculate the angle to the velocity vector to get the forward direction
        angle = math.atan2(self.vel.y, self.vel.x)
        other_points_angle = 0.75 * math.pi  # angle +- value to get the other two points in the triangle

        # Get the points of the triangle
        x0 = self.pos.x + BOID_SIZE * math.cos(angle)
        y0 = self.pos.y + BOID_SIZE * math.sin(angle)

        x1 = self.pos.x + BOID_SIZE * math.cos(angle + other_points_angle)
        y1 = self.pos.y + BOID_SIZE * math.sin(angle + other_points_angle)

        x2 = self.pos.x + BOID_SIZE * math.cos(angle - other_points_angle)
        y2 = self.pos.y + BOID_SIZE * math.sin(angle - other_points_angle)

        # Draw
        pg.draw.polygon(screen, WHITE, [(x1, y1), (x2, y2), (x0, y0)])


# Helper functions
def remap(n, start1, stop1, start2, stop2):
    """Remap a value in one range to a different range"""
    new_value = (n - start1) / (stop1 - start1) * (stop2 - start2) + start2
    if start2 < stop2:
        return constrain(new_value, start2, stop2)
    else:
        return constrain(new_value, stop2, start2)


def constrain(n, low, high):
    """Constrain a value to a range"""
    return max(min(n, high), low)


def limit(vector, length):
    """Cap a value"""
    if vector.length_squared() <= length * length:
        return
    else:
        vector.scale_to_length(length)


if __name__ == '__main__':
    sim = Simulation()
    sim.run()
