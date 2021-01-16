import datetime
import gym
import gym.utils.seeding
import numpy as np
import pybullet

# qibullet imports
from qibullet import SimulationManager

class EnvBase(gym.Env):
    """ ENV base to handle the quibullet simulation handler and other basic env
    """
    metadata = {"render.modes": ["human", "rgb_array"]}


    def __init__(self, render = False):
        self.if_rendered = render
        # TODO: create a video saving for nao


    def initialize_scene_and_robot(self):

        # start the simulation manager
        self.simulation_manager = SimulationManager()
        self._p = self.simulation_manager.launchSimulation(gui = self.if_rendered)

        self.robot = self.simulation_manager.spaqnNao(self._p, spwan_ground_plane = True)

        self._debug_visualizer()

        self.state_id = self._p.saveState()

    def render(self, mode="human"):
        # Taken care of by pybullet
        if not self.is_rendered:
            self.is_rendered = True
            self.simulation_manager.stopSimulation(self._p)
            self.initialize_scene_and_robot()
            self.reset()

        if mode != "rgb_array":
            return np.array([])

        yaw, pitch, dist, lookat = self._p.getDebugVisualizerCamera()[-4:]

        view_matrix = self._p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=lookat,
            distance=dist,
            yaw=yaw,
            pitch=pitch,
            roll=0,
            upAxisIndex=2,
        )
        proj_matrix = self._p.computeProjectionMatrixFOV(
            fov=60,
            aspect=float(self._render_width) / self._render_height,
            nearVal=0.1,
            farVal=100.0,
        )
        (_, _, px, _, _) = self._p.getCameraImage(
            width=self._render_width,
            height=self._render_height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=self._p.ER_BULLET_HARDWARE_OPENGL,
        )
        rgb_array = np.array(px)
        rgb_array = np.reshape(
            np.array(px), (self._render_height, self._render_width, -1)
        )
        rgb_array = rgb_array[:, :, :3]
        return rgb_array.astype(np.uint8)

    def close(self):
        self.simulation_manager.stopSimulation(self._p)

    def reset(self):
        # use simuilation manager reset simualtion.
        raise NotImplementedError

    def step(self):
        raise NotImplementedError


    def seed(self, seed = None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    # TODO: implement the keyboard handler class
    def _debug_visualizer(self):
        pc = self._p
        pc.configureDebugVisualizer(pc.COV_ENABLE_RENDERING, 0)
        pc.configureDebugVisualizer(pc.COV_ENABLE_GUI, 0)
        pc.configureDebugVisualizer(pc.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
        pc.configureDebugVisualizer(pc.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        pc.configureDebugVisualizer(pc.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        pc.configureDebugVisualizer(pc.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
