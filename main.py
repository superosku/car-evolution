import math
import random
import Box2D

import pygame
from Box2D import b2World, b2EdgeShape, b2CircleShape

black = pygame.color.Color(0, 0, 0)
green = pygame.color.Color(0, 255, 0)
red = pygame.color.Color(255, 0, 0)
blue = pygame.color.Color(0, 0, 255)
yellow = pygame.color.Color(0, 255, 255)


def vec_len(vec):
    return math.sqrt(vec[0] ** 2 + vec[1] ** 2)


class Camera:
    def __init__(self, screen_x, screen_y):
        self.zoom = 100
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
            ground_points[-1][0] + 1 + random.random() * 1.5,
            ground_points[-1][1] + (random.random() - 0.49) * math.sqrt(i),
        ))
    ground_points.insert(0, (
        ground_points[0][0],
        -10
    ))
    ground_points.append((
        ground_points[-1][0],
        -10
    ))
    return ground_points


def random_car_points():
    car_points = [
        (
            random.random() * 2 - 1,
            random.random() * 2 - 1,
            random.random() * 0.2 + 0.05,
            random.random() < 0.5
        )
        for i in range(5)
    ]
    return car_points


class Simulation:
    def __init__(self, ground_points, car_points):
        self.world = b2World(
            gravity=(0, -10),
            doSleep=False
        )

        self.ground = self.world.CreateStaticBody()
        self.ground.CreateChainFixture(
            vertices=ground_points
        )

        self.car_points = car_points
        self.car_polygon_fixtures = []
        self.car_tires = []

        car_start_pos = (5, 3)

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
                density=1,
                friction=0.3,
                vertices=ordered_xys
            )
        )

        for x, y, radius, is_motor in car_points:
            if is_motor:
                tire = self.world.CreateDynamicBody(
                    position=(
                        car_start_pos[0] + x,
                        car_start_pos[1] + y
                    ),
                )
                tire.CreateCircleFixture(
                    radius=radius,
                    density=1,
                    friction=0.7,
                    pos=(0, 0)
                )
                self.world.CreateRevoluteJoint(
                    bodyA=car_body,
                    bodyB=tire,
                    anchor=tire.position,
                    motorSpeed=-50,
                    enableMotor=True,
                    maxMotorTorque=1000,
                )
                self.car_tires.append(tire)

    def step(self):
        self.world.Step(1.0 / 60, 8, 3)

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
                5
            )



def main():
    screen_size = (1000, 800)

    pygame.init()
    screen = pygame.display.set_mode(screen_size)
    clock = pygame.time.Clock()

    ground_points = random_ground_points()

    simulation = Simulation(
        ground_points,
        random_car_points()
    )

    camera = Camera(*screen_size)

    # simulation.draw()

    while True:
        clock.tick()

        try:
            pygame.display.set_caption(f'UPS: {int(clock.get_fps())}')
        except OverflowError:
            pass

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return
                if event.key == pygame.K_r:
                    while True:
                        try:
                            simulation = Simulation(
                                ground_points,
                                random_car_points()
                            )
                            break
                        except AssertionError:
                            pass
            if event.type == pygame.MOUSEWHEEL:
                breakpoint()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    camera.zoom *= 1.1
                if event.button == 5:
                    camera.zoom *= 0.9

        if not pygame.key.get_pressed()[pygame.K_h]:
            simulation.step()
        simulation.update_camera(camera)

        screen.fill(black)
        simulation.draw_ground(screen, camera)
        simulation.draw_car(screen, camera)

        if not pygame.key.get_pressed()[pygame.K_f]:
            pygame.display.flip()


if __name__ == '__main__':
    # random.seed(0)
    main()
