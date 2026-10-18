"""Check actual-cloud planning geometry without importing unbuilt ROS messages."""
import ast
from pathlib import Path
import numpy as np
from scipy import ndimage
import benchmark as b

source=Path(__file__).with_name('ros_flight.py')
tree=ast.parse(source.read_text(encoding='utf-8'))
node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LiveWorld')
namespace={'b':b,'np':np,'ndimage':ndimage}
exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),namespace)
cloud=b.generate_map(511)[2]
world=namespace['LiveWorld'](cloud)
literal=np.zeros((500,500),bool)
for x,y,z in cloud:
    if -25<=x<=25 and -25<=y<=25:
        gx=int((float(x)+25)/.1);gy=int((float(y)+25)/.1)
        if 0<=gx<500 and 0<=gy<500:literal[gy,gx]=True
assert np.array_equal(world.obstacle,literal)
contained=((cloud[:,None,:]>=world.boxes[None,:,:3])&(cloud[:,None,:]<=world.boxes[None,:,3:])).all(axis=2).any(axis=1)
assert contained.all(),'Received obstacle points escaped navigation bounding boxes'
for variant in ['A','B','C']:
    routes,changes=b.initial_routes(world,variant)
    tasks=[v for r in routes for v in r]
    assert sorted(tasks)==list(range(6,6+len(world.targets)))
    print(variant,'targets',len(tasks),'ownership changes',changes,'estimated routes',[round(b.route_cost(world,i,r),2) for i,r in enumerate(routes)])
print('Live geometry checks passed: actual obstacle samples contained, exact teacher obstacle denominator preserved, every task has exactly one initial owner.')
