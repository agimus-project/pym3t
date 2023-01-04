import numpy as np
import pyicg

t = pyicg.Tracker('tracker')
t.SetUp(True)

opt = pyicg.Optimizer('optimizer')
opt = pyicg.Optimizer('optimizer', 'optimizer_path')
opt.name = 'Yop'
opt.metafile_path = 'Yep'
opt.tikhonov_parameter_rotation = 0.5
opt.tikhonov_parameter_translation = 0.6

print(opt.name)
print(opt.metafile_path)
print(opt.tikhonov_parameter_rotation)
print(opt.tikhonov_parameter_translation)

T_w_b = np.eye(4)
body = pyicg.Body(name='body_name', geometry_path='body_path')
# body = pyicg.Body(name='toto', 
#             geometry_path='toto_path', 
#             geometry_unit_in_meter=1.0, 
#             geometry_counterclockwise=True, 
#             geometry_enable_culling=True,
#             geometry2body_pose=pose, 
#             silhouette_id=0)
body.body2world_pose = T_w_b
print(body.body2world_pose)