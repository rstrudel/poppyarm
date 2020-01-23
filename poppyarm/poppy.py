import os
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

from poppyarm.robot import RobotArm


class Poppy(RobotArm):
    def __init__(self):
        current_dir = os.path.dirname(__file__)
        robot_dir = os.path.join(current_dir, "assets/")
        urdf_path = os.path.join(current_dir, "assets/poppy_ergo_jr.urdf")
        self.wrapper = RobotWrapper.BuildFromURDF(urdf_path, package_dirs=[robot_dir])
        self.dummy_data = self.wrapper.data.copy()
        self._set_collision_pairs(self.wrapper)
        # do not count the universe as a joint
        self.joints_shape = (self.wrapper.model.njoints - 1, 3)

    def _set_collision_pairs(self, robot):
        model = robot.model
        collision_model = robot.collision_model
        collision_model.addAllCollisionPairs()
        pairs_to_remove = []
        # remove all pairs of adjacent bodies
        for col_pair in collision_model.collisionPairs:
            obj1_id = col_pair.first
            obj2_id = col_pair.second

            obj1 = collision_model.geometryObjects[obj1_id]
            obj2 = collision_model.geometryObjects[obj2_id]

            remove_col_pair = False
            if obj1.parentJoint == obj2.parentJoint:
                remove_col_pair = True

            if obj1.parentJoint < obj2.parentJoint:
                if obj1.parentJoint == model.parents[obj2.parentJoint]:
                    remove_col_pair = True
            else:
                if obj2.parentJoint == model.parents[obj1.parentJoint]:
                    remove_col_pair = True

            if remove_col_pair:
                pairs_to_remove.append(col_pair)

        # collision_pairs indices are updated after each removal
        # remove bigger indices first to avoid reindexing issues
        for col_pair in pairs_to_remove[::-1]:
            collision_model.removeCollisionPair(col_pair)
        robot.collision_model = collision_model
        robot.collision_data = robot.collision_model.createData()

    def random_configuration(self):
        model = self.wrapper.model
        is_colliding = True
        while is_colliding:
            theta = np.random.uniform(0, 2 * np.pi, size=(6,))
            q = self.theta_to_q(theta)
            is_colliding = self.is_auto_colliding(q)
        return theta, q

    def reset(self):
        self.theta, q = self.random_configuration()
        self._apply_fk(q)

    def theta_to_q(self, theta):
        """
        Returns (cos, sin) representation for each of the joints
        """
        q = np.concatenate(
            (np.cos(theta)[..., None], np.sin(theta)[..., None]), axis=-1
        )
        q = q.reshape(-1, 1)
        return q

    def is_auto_colliding(self, q):
        """
        Check if the configuration is autocolliding
        """
        model = self.wrapper.model
        data = self.dummy_data
        collision_model = self.wrapper.collision_model
        collision_data = self.wrapper.collision_data
        is_colliding = True
        is_colliding = pin.computeCollisions(
            model, data, collision_model, collision_data, q, True
        )
        return is_colliding

    def move(self, dtheta):
        """
        Returns a boolean indicating is the robot was able to move or not
        """
        theta = self.theta + dtheta
        q = self.theta_to_q(theta)
        collision = self.is_auto_colliding(q)
        if not collision:
            self._apply_fk(q)
            self.theta = theta
        return not collision
