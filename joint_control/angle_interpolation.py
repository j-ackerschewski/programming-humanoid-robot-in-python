'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello, leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.time = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        # get the current time the robot is alive
        if self.time == 0:
            self.time = perception.time
        curTime = perception.time - self.time

        # go through all joints
        for jointIndex in range(len(keyframes[0])):
            for timeIndex in range(len(keyframes[1][jointIndex])):
                if timeIndex < len(keyframes[1][jointIndex]) - 1 and \
                        keyframes[1][jointIndex][timeIndex] < curTime < keyframes[1][jointIndex][timeIndex + 1]:
                    # do bezier interpolation
                    t = (curTime - keyframes[1][jointIndex][timeIndex]) / (keyframes[1][jointIndex][timeIndex + 1] - keyframes[1][jointIndex][timeIndex])
                    p0 = np.array([keyframes[1][jointIndex][timeIndex], keyframes[2][jointIndex][timeIndex][0]])
                    p3 = np.array([keyframes[1][jointIndex][timeIndex + 1], keyframes[2][jointIndex][timeIndex + 1][0]])
                    p1 = p0 + np.array([keyframes[2][jointIndex][timeIndex][1][1], keyframes[2][jointIndex][timeIndex][1][2]])
                    p2 = p0 + np.array([keyframes[2][jointIndex][timeIndex][2][1], keyframes[2][jointIndex][timeIndex][2][2]])
                    b = (((1 - t) ** 3) * p0) + (3 * ((1 - t) ** 2) * t * p1) + (3 * (1 - t) * (t ** 2) * p2) + ((t ** 3) * p3)
                    target_joints[keyframes[0][jointIndex]] = b[1]

        # i am not entirely sure if the calculation is correct, but the keyframes are reached and it is interpolated.
        # It just feels like the robot moves quite fast between the keyframes. The points might be selected wrongly
        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
