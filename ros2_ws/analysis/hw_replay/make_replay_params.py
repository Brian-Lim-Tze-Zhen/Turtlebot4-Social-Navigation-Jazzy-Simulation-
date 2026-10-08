#!/usr/bin/env python3
"""make_replay_params.py <nav2 config> <out yaml> - controller-only params for the critic replay (8 Oct 2026).

Keeps controller_server (MPPI + SocialCritic exactly as in the config) and builds a minimal local costmap
(rolling window in odom: obstacle layer on /scan + inflation, same footprint), so the controller runs on a
robot bag without map server, filters or person-cloud layers. The SocialCritic decisions (lane, side) do not
depend on the costmap content. The progress checker is relaxed: the robot's motion comes from the bag.
"""
import sys, yaml

d = yaml.safe_load(open(sys.argv[1]))
cs = d['controller_server']['ros__parameters']
cs['use_sim_time'] = True
pc = cs.get('progress_checker', {})
pc['required_movement_radius'] = 0.0
pc['movement_time_allowance'] = 1000.0
cs['progress_checker'] = pc
src = d['local_costmap']['local_costmap']['ros__parameters']
lc = {k: v for k, v in src.items() if not isinstance(v, dict)}
lc.update({'use_sim_time': True, 'global_frame': 'odom', 'robot_base_frame': 'base_link',
           'plugins': ['obstacle_layer', 'inflation_layer']})
lc.pop('filters', None)
lc['obstacle_layer'] = src.get('obstacle_layer', {
    'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True, 'observation_sources': 'scan',
    'scan': {'topic': 'scan', 'marking': True, 'clearing': True, 'data_type': 'LaserScan'}})
lc['inflation_layer'] = src.get('inflation_layer', {'plugin': 'nav2_costmap_2d::InflationLayer'})
out = {'controller_server': {'ros__parameters': cs},
       'local_costmap': {'local_costmap': {'ros__parameters': lc}},
       'replay_lifecycle_manager': {'ros__parameters': {'use_sim_time': False, 'autostart': True,
                                                        'node_names': ['controller_server'],
                                                        'bond_timeout': 0.0}}}
yaml.safe_dump(out, open(sys.argv[2], 'w'), sort_keys=False)
print('wrote', sys.argv[2])
