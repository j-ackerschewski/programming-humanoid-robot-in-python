'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
import numpy as np
from scipy.linalg import pinv
from math import atan2

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = {}
        # YOUR CODE HERE
        max_step = 0.1
        lambda_ = 1

        # set angles to current values. All values have to be set, since they are called in forward_kinematics
        for group in self.chains:
            for x in self.chains[group]:
                joint_angles[x] = self.perception.joint[x]

        target = np.array([self.from_trans(transform)]).T

        for i in range(1000):
            self.forward_kinematics(joint_angles)
            Ts = np.array([self.transforms[x] for x in self.chains[effector_name]])
            Te = np.array([self.from_trans(Ts[-1])]).T
            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            T = np.array([self.from_trans(x) for x in Ts]).T
            J = Te - T
            dT = Te - T
            J[0, :] = -dT[1, :]
            J[1, :] = dT[0, :]
            J[-1, :] = 1
            d_theta = lambda_ * np.dot(pinv(J), e)
            for y, x in enumerate(self.chains[effector_name]):
                joint_angles[x] += np.array(d_theta.T)[0][y]

            if np.linalg.norm(d_theta) < 1e-4:
                break
        print "calc finished"
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        angles = self.inverse_kinematics(effector_name, transform)
        self.keyframes = (self.chains[effector_name], [[0, 5]] * len(self.chains[effector_name]),
                          [[[self.perception.joint[name], [3, -0.25, 0], [3, 0, 0]], [angles[name], [3, 0, 0], [3, 0, 0]]] for name in self.chains[effector_name]])  # the result joint angles have to fill in

    def from_trans(self, m):
        angle_x, angle_y, angle_z = 0, 0, 0
        if m[0, 0] == 1:
            angle_x = atan2(m[2][1], m[1][1])
        if m[1, 1] == 1:
            angle_y = atan2(m[0][2], m[0][0])
        if m[2, 2] == 1:
            angle_z = atan2(m[0][1], m[0][0])
        return [m[-1][0], m[-1][1], m[-1][2], angle_x, angle_y, angle_z]


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = np.identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
