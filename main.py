import math
import random

import pygame
from Box2D import b2World

black = pygame.color.Color(0, 0, 0)
green = pygame.color.Color(0, 255, 0)
red = pygame.color.Color(255, 0, 0)
blue = pygame.color.Color(0, 0, 255)
yellow = pygame.color.Color(0, 255, 255)


car_start_pos = (5, 3)

kill_count = 10
total_car_count = 100
mutate_factor = 0.1
time_in_seconds = 30


def vec_len(vec):
    return math.sqrt(vec[0] ** 2 + vec[1] ** 2)


def sigmoid(x):
  return 1 / (1 + math.exp(-x))


class Camera:
    def __init__(self, screen_x, screen_y):
        self.zoom = 40
        self.x = 0
        self.y = 0

        self.screen_x = screen_x
        self.screen_y = screen_y

    def box2d_to_pygame(self, x, y):
        return (
            int(self.screen_x / 2 + x * self.zoom - self.x * self.zoom),
            int(self.screen_y / 2 - y * self.zoom + self.y * self.zoom)
        )


def random_ground_points():
    ground_points = [(0, 0), (10, 0)]
    for i in range(200):
        ground_points.append((
            ground_points[-1][0] + 0.7 + random.random() * 1,
            ground_points[-1][1] + (random.random() - 0.52) * math.sqrt(i) * 0.2,
        ))
    ground_points.insert(0, (
        ground_points[0][0],
        -100
    ))
    ground_points.append((
        ground_points[-1][0],
        -100
    ))
    return ground_points


def random_car_points():
    car_pointss = [
        (
            random.random() * 2 - 1,
            random.random() * 2 - 1,
            # random.random() * 0.2 + 0.05,
            random.random() * 6 - 3, #  Used for sigmoid later
            # i < random.randrange(1, 4),
            i < 2,
            # random.random() < 0.5
        )
        for i in range(5)
    ]

    return car_pointss


def mutate(car_points):
    car_points = [
        (
            x + (random.random() - 0.5) * mutate_factor,
            y + (random.random() - 0.5) * mutate_factor,
            radius + (random.random() - 0.5) * mutate_factor,
            is_tire if random.random() > 0.10 else not is_tire
        )
        for x, y, radius, is_tire
        in car_points
    ]
    if random.random() < 0.10 and len(car_points) > 3:
        random.shuffle(car_points)
        car_points = car_points[0:-1]
    if random.random() < 0.10 and len(car_points) < 7:
        car_points = car_points + [random_car_points()[-1]]
    return car_points


class Simulation:
    def __init__(self, ground_points, car_points):
        self.world = b2World(
            gravity=(0, -10),
            doSleep=False
        )

        self.step_count = 0
        self.record = 0
        self.steps_since_record = 0

        self.ground = self.world.CreateStaticBody()
        self.ground.CreateChainFixture(
            vertices=ground_points,
            friction=0.7
        )

        self.car_points = car_points
        self.car_polygon_fixtures = []
        self.car_tires = []

        car_body = self.world.CreateDynamicBody(position=car_start_pos)
        self.car_body = car_body

        xys = [(point[0], point[1]) for point in car_points]
        xys = sorted(xys, key=lambda x: x[0])

        ordered_xys = [xys[0]]
        while True:
            # breakpoint()
            if len(ordered_xys) > 1:
                vector = (
                    ordered_xys[-1][0] - ordered_xys[-2][0],
                    ordered_xys[-1][1] - ordered_xys[-2][1],
                )
            else:
                vector = (0, 1)
            # breakpoint()

            min_angle = None
            min_xy = None

            for xy in xys:
                if xy not in ordered_xys[1:]:
                    second_vector = (
                        xy[0] - ordered_xys[-1][0],
                        xy[1] - ordered_xys[-1][1],
                    )

                    try:
                        thing = (
                            vector[0] * second_vector[0] +
                            vector[1] * second_vector[1]
                        ) / (vec_len(vector) * vec_len(second_vector))
                        thing = max(-1, thing)
                        thing = min(1, thing)
                        angle = math.acos(thing)
                    except ZeroDivisionError:
                        continue

                    if min_angle is None or angle < min_angle:
                        min_angle = angle
                        min_xy = xy

            if min_xy in ordered_xys:
                break

            ordered_xys.append(min_xy)

        self.car_polygon_fixtures.append(
            car_body.CreatePolygonFixture(
                density=10,
                friction=0.7,
                vertices=ordered_xys
            )
        )

        for x, y, tire_stats, is_motor in car_points:
            if is_motor:
                tire = self.world.CreateDynamicBody(
                    position=(
                        car_start_pos[0] + x,
                        car_start_pos[1] + y
                    ),
                )

                normalized_tire_stats = sigmoid(tire_stats)

                radius = normalized_tire_stats * 0.5
                friction = normalized_tire_stats
                motor_speed = -70 * (1 - normalized_tire_stats)
                motor_torque = 1000 * normalized_tire_stats

                tire.CreateCircleFixture(
                    radius=radius,
                    density=10,
                    friction=friction,
                    pos=(0, 0)
                )
                self.world.CreateRevoluteJoint(
                    bodyA=car_body,
                    bodyB=tire,
                    anchor=tire.position,
                    motorSpeed=motor_speed,
                    enableMotor=True,
                    maxMotorTorque=motor_torque,
                )
                self.car_tires.append(tire)

    def step(self):
        self.world.Step(1.0 / 60, 8, 3)

        self.step_count += 1
        self.steps_since_record += 1

        if self.score() > self.record:
            self.record = self.score()
            self.steps_since_record = 0

    def update_camera(self, camera):
        camera.x = self.car_body.position[0]
        camera.y = self.car_body.position[1]

    def draw_ground(self, screen, camera):
        # Ground
        ground_points = [
            camera.box2d_to_pygame(x, y)
            for x, y in
            self.ground.fixtures[0].shape.vertices
        ]
        # breakpoint()
        pygame.draw.lines(
            screen,
            green,
            False,
            ground_points,
            2
        )

    def score(self):
        return self.car_body.position[0]

    def draw_car(self, screen, camera):
        # Car
        for tire in self.car_tires:
            x, y = tire.position
            radius = tire.fixtures[0].shape.radius
            angle = tire.angle

            pygame.draw.circle(
                screen,
                red,
                camera.box2d_to_pygame(x, y),
                int(camera.zoom * radius)
            )
            pygame.draw.line(
                screen,
                black,
                camera.box2d_to_pygame(x, y),
                camera.box2d_to_pygame(
                    x + math.cos(angle) * radius,
                    y + math.sin(angle) * radius
                ),
            )

        car_x, car_y = self.car_body.position
        car_angle = self.car_body.angle

        # Car triangles
        for triangle_fixture in self.car_polygon_fixtures:

            screen_points = []
            for x, y in triangle_fixture.shape.vertices:
                point_radius = math.sqrt(x * x + y * y)
                point_angle = math.atan2(y, x) - math.pi / 2
                screen_points.append(
                    camera.box2d_to_pygame(
                        car_x - math.sin(car_angle + point_angle) * point_radius,
                        car_y + math.cos(car_angle + point_angle) * point_radius,
                    )
                )

            pygame.draw.lines(
                screen,
                green,
                True,
                screen_points,
                2
            )

        for thing in self.car_points:
            point_x = thing[0]
            point_y = thing[1]

            point_radius = math.sqrt(point_x * point_x + point_y * point_y)
            point_angle = math.atan2(point_y, point_x) - math.pi / 2

            pygame.draw.circle(
                screen,
                yellow,
                camera.box2d_to_pygame(
                    car_x - math.sin(car_angle + point_angle) * point_radius,
                    car_y + math.cos(car_angle + point_angle) * point_radius,
                ),
                int(0.03 * camera.zoom)
            )


class Game:
    def __init__(self):
        self.screen_size = (600, 400)

        pygame.init()

        self.screen = pygame.display.set_mode(
            self.screen_size,
            # flags=pygame.DOUBLEBUF | pygame.HWSURFACE
        )
        self.clock = pygame.time.Clock()

        self.camera = Camera(*self.screen_size)

        self.ground_points = random_ground_points()

        self.tick = 0

        self.car_pointss = [
            random_car_points()
            for _ in range(total_car_count)
        ]
        #self.car_pointss = [random_car_points()] * total_car_count
        self.simulations =[]

        self.fast_mode = False
        self.show_only_one = False

        self.init_simulations()

    def init_simulations(self):
        self.simulations = [
            Simulation(
                self.ground_points,
                car_points
            ) for car_points in self.car_pointss
        ]

    def draw(self):
        self.screen.fill(black)
        self.simulations[0].draw_ground(self.screen, self.camera)
        if self.show_only_one:
            self.simulations[0].draw_car(self.screen, self.camera)
        else:
            for simulation in self.simulations:
                simulation.draw_car(self.screen, self.camera)
        pygame.display.flip()

    def simulation_tick(self):
        for simulation in self.simulations:
            if not pygame.key.get_pressed()[pygame.K_h]:
                simulation.step()

        # sorted_simulations = sorted(self.simulations, key=lambda x: -x.score())
        # simulation = sorted_simulations[0]
        # simulation.update_camera(self.camera)
        self.simulations[0].update_camera(self.camera)

        if not self.fast_mode:
            self.draw()

    def mutate(self):
        data = sorted([
            (car_points, simulation.score())
            for car_points, simulation in
            zip(self.car_pointss, self.simulations)
        ], key=lambda x: -x[1])

        print('best score:', data[0][1])

        surviving = [item for item, score in data[0:-kill_count]]

        for i in range(kill_count):
            surviving.append(
                mutate(
                    surviving[i]
                    if kill_count * 2 < total_car_count else
                    surviving[0]
                )
            )

        self.car_pointss = surviving

        self.init_simulations()

    def simulate(self):
        round_number = 1
        while True:
            self.clock.tick()

            try:
                pygame.display.set_caption(f'UPS: {int(self.clock.get_fps())} round: {round_number}')
            except OverflowError:
                pass

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        return
                    if event.key == pygame.K_r:
                        self.init_simulations()
                    if event.key == pygame.K_f:
                        self.fast_mode = not self.fast_mode
                    if event.key == pygame.K_o:
                        self.show_only_one = not self.show_only_one
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 4:
                        self.camera.zoom *= 1.1
                    if event.button == 5:
                        self.camera.zoom *= 0.9

            self.simulation_tick()

            self.tick += 1

            if self.tick % (60 * time_in_seconds) == 0:
                if self.fast_mode:
                    self.draw()
                self.mutate()
                round_number += 1


if __name__ == '__main__':
    game = Game()
    game.simulate()

