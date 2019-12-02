import numpy as np
import pinocchio as pin
pin.switchToNumpyArray()


class RobotArm:
    def __init__(self):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError

    def is_auto_colliding(self, theta):
        raise NotImplementedError

    def get_joints_pos(self, data):
        if data is None:
            data = self.data
        joints_pos = np.zeros((self.n_joints, 2))
        for i in range(self.n_joints):
            joints_pos[i] = data.oMi[i + 1].np[:2, -1]
        return joints_pos

    def _apply_fk(self, q):
        pin.framesForwardKinematics(self.wrapper.model, self.wrapper.data, q[:, None])

    def get_oMi(self, q):
        pin.forwardKinematics(self.wrapper.model, self.dummy_data, q[:, None])
        return self.dummy_data.oMi

    def get_oMf(self, q):
        pin.framesForwardKinematics(self.wrapper.model, self.dummy_data, q[:, None])
        return self.dummy_data.oMf

    def get_joints_pos(self, q=None):
        if q is not None:
            oMi = self.get_oMi(q)
        else:
            oMi = self.wrapper.data.oMi

        n_joints, dim_joint = self.joints_shape
        joints_pos = np.zeros((n_joints, dim_joint))
        for i in range(self.joints_shape[0]):
            joints_pos[i] = oMi[i + 1].np[:dim_joint, -1]
        return joints_pos

    def get_bodies_pos(self, q=None):
        if q is not None:
            oMf = self.get_oMf(q)
        else:
            oMf = self.wrapper.data.oMf

        bodies_pos = np.zeros((len(oMf), 3))
        for i in range(len(oMf)):
            bodies_pos[i] = oMf[i].translation
        return bodies_pos
