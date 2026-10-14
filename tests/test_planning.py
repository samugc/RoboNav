import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/entrega_mapas_package'))
import unittest
import numpy as np
from entrega_mapas_package.planning import ThetaStar
class PlanningTests(unittest.TestCase):
    def setUp(self): self.p=ThetaStar()
    def test_open_map_path(self):
        grid=np.zeros((8,8),dtype=int)
        route,count=self.p.plan(grid,(0,0),(7,7))
        self.assertEqual(route[0],(0,0));self.assertEqual(route[-1],(7,7))
        self.assertGreater(count,0)
        for a,b in zip(route,route[1:]):self.assertTrue(self.p.line_of_sight(a,b,grid))
    def test_blocked_map(self):
        grid=np.zeros((8,8),dtype=int);grid[4,:]=1
        route,_=self.p.plan(grid,(0,0),(7,7));self.assertIsNone(route)
    def test_invalid_start_has_consistent_return(self):
        grid=np.zeros((4,4),dtype=int);grid[0,0]=1
        self.assertEqual(self.p.plan(grid,(0,0),(3,3)),(None,0))
        self.assertEqual(self.p.plan(grid,(-1,0),(3,3)),(None,0))
    def test_visibility_obstacle(self):
        grid=np.zeros((5,5),dtype=int);grid[2,2]=1
        self.assertFalse(self.p.line_of_sight((0,0),(4,4),grid))
        self.assertFalse(self.p.line_of_sight((4,4),(0,0),grid))
    def test_same_start_goal(self):
        route,_=self.p.plan(np.zeros((3,3)),(1,1),(1,1))
        self.assertEqual(route,[(1,1)])
if __name__=='__main__':unittest.main()
