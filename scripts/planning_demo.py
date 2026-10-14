"""Small synthetic experiment; no ROS, simulator, images or plotting required."""
import json, sys, math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/entrega_mapas_package'))
import numpy as np
from entrega_mapas_package.planning import ThetaStar
planner=ThetaStar()
grid=np.zeros((12,12),dtype=int);grid[6,:10]=1
route,pops=planner.plan(grid,(1,1),(10,10))
print(json.dumps(dict(scenario='synthetic-wall-with-gap',path=route,
    queue_pops=pops,path_length_cells=sum(math.dist(a,b) for a,b in zip(route,route[1:])) if route else None),indent=2))
