from pyicg import *

t = Tracker('tracker')
t.SetUp(True)

opt = Optimizer('optimizer')
opt.name = 'Yop'
opt.metafile_path = 'Yep'
opt.tikhonov_parameter_rotation = 0.5
opt.tikhonov_parameter_translation = 0.6

print(opt.name)
print(opt.metafile_path)
print(opt.tikhonov_parameter_rotation)
print(opt.tikhonov_parameter_translation)