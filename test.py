from pyicg import *

t = Tracker('tracker')
t.SetUp(True)

opt = Optimizer('optimizer')
print(opt.name)
# print(opt.metafile_path)  # BUG: impossible to convert
print(opt.tikhonov_parameter_rotation)
print(opt.tikhonov_parameter_translation)