import numpy as np
import matplotlib.pyplot as plt

import pybullet as p

class CassieState(object):
    def __init__(self, cassie_id):
        self.cassie_id = cassie_id
        self.joint_id = []
        self._get_id()

    def _get_id(self):
        self.axis_store = []
        for i in range(p.getNumJoints(self.cassie_id)):
            info = p.getJointInfo(self.cassie_id, i)
            axis = np.array(list(info[-4]))
            where_ = np.where(axis != 0)[0]
            if where_.size != 0:
                self.joint_id.append({'id': i,
                                      'name':info[1].decode('utf-8'),
                                      'body_name': info[-4]})
                self.axis_store.append(where_[0])
                if self.axis_store[-1] == 0:
                    print('prismatic:', info[1])
                else:
                    print('revolute:', info[1])

        print(self.axis_store)
        self._get_state()

    def _get_state(self):
        qpos, short_pos = self._get_qpos()
        # qvel = self._get_qvel()
        print(len(qpos), len(short_pos))
        print(self.axis_store)

    def _get_qpos(self):
        pos = []
        short_pos = []
        # parsing the joints to get state
        base_pos = p.getBasePositionAndOrientation(self.cassie_id)
        # base position
        pos.extend(list(base_pos[0]))
        pos.extend(list(base_pos[1]))
        # short_base_position
        short_pos.extend(list(base_pos[0]))
        short_pos.extend(list(base_pos[1]))


        for i in range(len(self.joint_id)):
            info = p.getJointInfo(self.cassie_id, self.joint_id[i]['id'])
            positions = list(info[-3])
            orientations = list(info[-2])
            # position
            pos.extend(positions)
            # orientation
            pos.extend(orientations)
            # short state
            short_pos.append(positions[self.axis_store[i]])
            short_pos.append(orientations[self.axis_store[i]])

        return pos, short_pos

    def _get_qvel(self):
        pass




