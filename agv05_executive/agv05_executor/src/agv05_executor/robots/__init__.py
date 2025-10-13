from __future__ import absolute_import
from __future__ import unicode_literals

from .robot import Robot

robots = {
    'tracked_agv05': 'agv05_executor.robots.tracked_agv05.TrackedAgv05',
    'trackless_agv05': 'agv05_executor.robots.trackless_agv05.TracklessAgv05',
    'tracked_sim': 'agv05_executor.robots.tracked_sim.TrackedSim',
    'trackless_sim': 'agv05_executor.robots.trackless_sim.TracklessSim',
    'trackless_simulator': 'agv05_executor.robots.trackless_simulator.TracklessSimulator',
}
