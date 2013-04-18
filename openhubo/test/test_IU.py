import unittest
import openhubo.IU as IU

class TestLoading(unittest.TestCase):
    def test_ladder_build(self):
        gen=IU.IULadderGenerator()
        gen.load_parameters('parameters/70_0.20.iuparam')
        gen.make_ladder()
        gen.make_ladder_env()
        gen.print_parameters()
        gen2=IU.IULadderGenerator('parameters/80_0.25.iuparam')
        gen2.make_ladder_env()
        gen2.print_parameters()

if __name__=='__main__':
    unittest.main(verbosity=2,testRunner=unittest.TextTestRunner())
