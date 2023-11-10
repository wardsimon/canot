# SPDX-License-Identifier: BSD-3-Clause
import dataclasses

# flake8: noqa F401

import numpy as np

from vendeeglobe import (
    Checkpoint,
    Heading,
    Instructions,
    Location,
    MapProxy,
    Vector,
    WeatherForecast,
    config,
)
from vendeeglobe.utils import distance_on_surface, goto as go_to
from dataclasses import dataclass
from random import random


@dataclass
class Position:
    latitude: float = 0.0
    longitude: float = 0.0


CREATOR = "CaptainHaddock"  # This is your team name

import os

dir_path = os.path.dirname(os.path.realpath(__file__))
icon = os.path.join(dir_path, "haddock.png")


class Bot:
    """
    This is the ship-controlling bot that will be instantiated for the competition.
    """

    def __init__(self):
        self.tacking_n = 0
        self.tacking_times = 0
        self.previous_tack = 0
        self.non_tacking = 0
        self.team = CREATOR  # Mandatory attribute
        self.avatar = icon  # Optional attribute
        self.previous = Position()
        self.step = 0
        self.tacking = False
        self.course = [
            Checkpoint(latitude=43.797109, longitude=-11.264905, radius=50),
            Checkpoint(latitude=39.71, longitude=-50.01, radius=50),
            Checkpoint(latitude=20.89, longitude=-70.741, radius=50),
            Checkpoint(latitude=20.138, longitude=-73.652, radius=50),
            Checkpoint(latitude=19.114, longitude=-75.421, radius=50),
            Checkpoint(longitude=-75.322, latitude=17.099, radius=50),
            Checkpoint(longitude=-80.0148, latitude=9.3935, radius=15),
            Checkpoint(longitude=-78.223, latitude=5.178, radius=50),
            Checkpoint(latitude=2.806318, longitude=-168.943864, radius=500.0),
            Checkpoint(latitude=-16.55, longitude=172.44, radius=30.0),
            Checkpoint(latitude=-31.73, longitude=168.4, radius=30.0),
            Checkpoint(latitude=-48.22, longitude=149.68, radius=30.0),
            Checkpoint(latitude=-36.739, longitude=112.324, radius=30.0),
            Checkpoint(latitude=-15.668984, longitude=77.674694, radius=1190.0),
            Checkpoint(latitude=-38.41, longitude=18.11, radius=200.0),
            Checkpoint(latitude=15.454, longitude=-20.522, radius=200.0),
            Checkpoint(latitude=35.246, longitude=-21.753, radius=50.0),
            Checkpoint(latitude=43.797109, longitude=-11.264905, radius=50),
            Checkpoint(
                latitude=config.start.latitude,
                longitude=config.start.longitude,
                radius=5,
            ),
        ]

    def run(
            self,
            t: float,
            dt: float,
            longitude: float,
            latitude: float,
            heading: float,
            speed: float,
            vector: np.ndarray,
            forecast: WeatherForecast,
            world_map: MapProxy,
    ):
        """
        This is the method that will be called at every time step to get the
        instructions for the ship.

        Parameters
        ----------
        t:
            The current time in hours.
        dt:
            The time step in hours.
        longitude:
            The current longitude of the ship.
        latitude:
            The current latitude of the ship.
        heading:
            The current heading of the ship.
        speed:
            The current speed of the ship.
        vector:
            The current heading of the ship, expressed as a vector.
        forecast:
            The weather forecast for the next 5 days.
        world_map:
            The map of the world: 1 for sea, 0 for land.
        """
        current_position = Position(latitude=latitude, longitude=longitude)

        instructions = Instructions()
        for ch in self.course:

            traveled_dist = distance_on_surface(
                longitude1=current_position.longitude,
                latitude1=current_position.latitude,
                longitude2=self.previous.longitude,
                latitude2=self.previous.latitude,
            )
            dist = distance_on_surface(
                longitude1=longitude,
                latitude1=latitude,
                longitude2=ch.longitude,
                latitude2=ch.latitude,
            )
            jump = dt * np.linalg.norm(speed)
            if dist < 2.0 * ch.radius + jump:
                instructions.sail = min(ch.radius / jump, 1)
            else:
                instructions.sail = 1.0
            if dist < ch.radius:
                ch.reached = True
            if not ch.reached:
                # if np.abs(traveled_dist) < 2 and self.step > 40 and self.course.index(ch) != len(self.course) - 1 and self.tacking_times < 4:
                #     print(traveled_dist)
                #     fudge = 2 * (random() - 0.5)
                #     a = go_to(Location(longitude=ch.longitude, latitude=ch.latitude),
                #               Location(longitude=longitude, latitude=latitude))
                #     if self.tacking and self.tacking_n < 15:
                #         self.tacking_n += 1
                #         instructions.heading = Heading(heading)
                #         break
                #     if self.previous_tack > 0:
                #         if fudge > 0:
                #             fudge = -fudge
                #     else:
                #         if fudge < 0:
                #             fudge = -fudge
                #     instructions.heading = Heading(angle=a * fudge)
                #     self.previous_tack = fudge
                #     self.tacking = True
                #     self.tacking_times += 1
                #     self.tacking_n = 0
                #     break
                self.tacking = False
                self.tacking_n = 0
                self.tacking_times = 0
                instructions.location = Location(
                    longitude=ch.longitude, latitude=ch.latitude
                )
                break
        self.previous = current_position
        self.step += 1
        return instructions
