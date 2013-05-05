from openravepy import Environment,RaveCreateCollisionChecker
from numpy import pi
from openhubo import Pose
import unittest

class TestCollisionMap(unittest.TestCase):

    def setUp(self):
        env=Environment()
        #env.SetViewer('qtcoin')
        env.Load('test_collisionmap.env.xml')

        robot=env.GetRobots()[0]

        self.pose=Pose(robot)
        self.env=env
        self.robot=robot

    def test_ode_self_collision(self):
        self.env.SetCollisionChecker(RaveCreateCollisionChecker(self.env,'ode'))
        self._iterate_over_joint()

    def test_pqp_self_collision(self):
        self.env.SetCollisionChecker(RaveCreateCollisionChecker(self.env,'pqp'))
        self._iterate_over_joint()

    def _iterate_over_joint(self):
        self.pose['LHR']=10*pi/180
        cols=[]
        for k in range(100):
            self.pose['LHP']=k*pi/180
            self.pose.send()
            cols.append(self.robot.CheckSelfCollision())
        self.assertTrue(max(cols))

if __name__=='__main__':
    unittest.main(verbosity=2,testRunner=unittest.TextTestRunner())
