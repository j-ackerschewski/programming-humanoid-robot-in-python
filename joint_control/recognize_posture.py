'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle
from os import listdir

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = listdir('robot_pose_data')  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        j = perception.joint
        # get the data from perception (same order as learning data)
        data = [j['LHipYawPitch'], j['LHipRoll'], j['LHipPitch'], j['LKneePitch'],
                j['RHipYawPitch'], j['RHipRoll'], j['RHipPitch'], j['RKneePitch'],
                perception.imu[0], perception.imu[1]]
        # load the trained model
        clf = pickle.load(open('robot_pose.pkl'))
        # predict
        self.posture = self.posture_classifier[clf.predict([data])[0]]
        # The result seems pretty satisfying
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
