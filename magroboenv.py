import numpy as np
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import math
from time import sleep
import logging
from datetime import datetime

import magroboenv.myconfig as myconfig
import magroboenv.MProbe as MProbe

def square(x):
    return x*x

def distance(nparray1, nparray2):
    sum = square(nparray1[0] - nparray2[0]) + square(nparray1[1] - nparray2[1]) + square(nparray1[2] - nparray2[2])
    return math.sqrt(sum)

class EnvSpec(object):
    def __init__(self, timestep_limit, id):
        self.timestep_limit = timestep_limit
        self.id = id
        
class MagRoboEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    
    def __init__(self):

        date_str=datetime.now().strftime('%Y%m%d-%H%M%S')
        logfile=myconfig.Config.LOGFILE + date_str + ".log"

        logging.basicConfig(filename=logfile, level=logging.DEBUG)

        self.spec = EnvSpec(timestep_limit = myconfig.Config.TIMESTEP_LIMIT, id=1)


        # observation is the x, y, z coordinate of the grid
        self.observation_space = spaces.Box(low=MProbe.MProbe.ob_low, high=MProbe.MProbe.ob_high)

	#Action Space => Current values
        if myconfig.Config.CURR_DEVIATE_ACTIVE == True:
            self.action_space = spaces.MultiDiscrete(MProbe.Current.deviate_action)
            # 0 -> no change; 1 -> +ve change; 2 -> -ve change
        else:
            self.action_space = spaces.Box(low=MProbe.Current.ac_low, high=MProbe.Current.ac_high)

        #initial condition
        self.state = None
        self.steps_beyond_done = None

        # simulation related variables
        self.seed()

        #        self.set_goal() inside reset()
        self.reset()
        #print(self.robot)

        
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Parameters
        ----------
        action :

        Returns
        -------
        ob, reward, episode_over, info : tuple
            ob (object) :
                an environment-specific object representing your observation of
                the environment.
            reward (float) :
                amount of reward achieved by the previous action. The scale
                varies between environments, but the goal is always to increase
                your total reward.
            episode_over (bool) :
                whether it's time to reset the environment again. Most (but not
                all) tasks are divided up into well-defined episodes, and done
                being True indicates the episode has terminated. (For example,
                perhaps the pole tipped too far, or you lost your last life.)
            info (dict) :
                 diagnostic information useful for debugging. It can sometimes
                 be useful for learning (for example, it might contain the raw
                 probabilities behind the environment's last state change).
                 However, official evaluations of your agent are not allowed to
                 use this for learning.
        """
        #print("ac={}".format(action))
        logging.debug("action={}".format(action))
        
        self._take_action(action)
        ob = self.state
        
        reward = self._get_reward()

        '''
        if reward == 4:
            done = True
        else:
            done = False'''
        
        if self.curr_dist >= 10.0 and self.count_ts >= 50:
            print(" Reset Reward:{}, TS={}".format(reward, self.count_ts))
            done = True
        elif self.curr_dist < 0.1:
            print(" Reset Goal Reward:{}, TS={}".format(reward, self.count_ts))
            done = True
        else:
            done = False

        info = {}
        
        return ob, reward, done, info

    def _take_action(self, action):

        for i in range(9):
            if math.isnan(action[i]):
                self.seed(0)
                return

        #change the current
        if myconfig.Config.CURR_DEVIATE_ACTIVE == True:
            MProbe.desired_current.set_all_sys_curr_deviate(action)
        else:
            MProbe.desired_current.set_all_sys_current(action)

        #sleep sometime before reading
        sleep_time = 1.0 / myconfig.Config.RUN_TIMES_PER_SEC
        sleep(sleep_time) #sleep in seconds
        
        #read the changed orientation
        self.state = MProbe.slave.read_sys_orientation()

        self.count_ts += 1

    def reset(self):
        #set goal
        self.set_goal()

        #generate random seed
        self.seed()

        self.count_ts = 0
        
        #read current orientation
        self.state = MProbe.slave.read_sys_orientation()
        ob = self.state

        #Find Distance b/w start & goal
        self.init_dist = MProbe.slave.find_distance(MProbe.goal)
        self.curr_dist = self.init_dist
        
        return np.array(ob)

    def set_goal(self):
        #MProbe.goal.set_random_xyz()
        MProbe.goal.set_random_dev_xyz(MProbe.slave)

            
    def _get_reward(self):

        self.last_dist = self.curr_dist

        self.curr_dist = MProbe.slave.find_distance(MProbe.goal)
        print("distance:{}".format(self.curr_dist))
        #print("goal: ({}, {}, {})".format(MProbe.goal.coordinate.x, MProbe.goal.coordinate.y, MProbe.goal.coordinate.z))
        logging.debug("distance:{}".format(self.curr_dist))
        
        
        """ Reward is given for XYZ. """
        '''
        if self.curr_dist == 0.0:
            return 4
        elif self.curr_dist < 1.0:
            return 3
        elif self.curr_dist < 5.0:
            return 2
        elif self.last_dist > self.curr_dist:
            return 1
        elif self.last_dist < self.curr_dist:
            return -1
        else:
            return 0
        '''
        return np.random.normal(self.curr_dist, 0.5)*10

    def render(self, mode='human', close=False):
        pass
    
    def close(self):
        pass
    

    
