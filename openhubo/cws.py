from openrave import CollisionReport

def perform_cws(robot):
    env=robot.GetEnv()
    report=CollisionReport()
    env.CheckCollisions(robot,report)
    for c in report.contacts:
        print c.pos
    
