from netlogo_doe import lhs_of_variables

output_file = '../experiments/version2/v2_pyDOE_lhs.txt'

# define the variables and their ranges for a given experiment
# version 2 variables
v2_variables = {
	'turn-radius': (2, 20),
	'uuv-speed': (1, 5),
	'nav-bearing-std': (0, 1),
	'nav-velocity-std-cm': (0, 1),
	'max-obs-dist': (1, 30),
	'obs-influence': (.5, 5),
	'sonar_ping_rate': (1, 31),
	'side_angle': (20, 120),
	'side_low_range': (20, 40),
	'side_hi_range': (200, 1000),
	'signal-factor': (0, 90),
	'forward_angle': (20, 90),
	'forward_lo_range': (0,5),
	'forward_hi_range': (40, 800),
	'classification-threshold-std': (0.1, 10),
}
# version 1 variables
# nav_variables = {
# 	'turn-radius': (2, 20),
# 	'uuv-speed': (1, 5),
# 	'nav-bearing-std': (0, 2),
# 	'nav-velocity-std-cm': (0, 2),
# 	'max-obs-dist': (1, 30),
# 	'obs-influence': (.5, 5),
# 	'sonar_ping_rate': (1, 61),
# 	'side_angle': (20, 120),
# 	'side_low_range': (20, 40),
# 	'side_hi_range': (200, 1000),
# 	'side_p_detect': (0.2, 1),
# 	'forward_angle': (20, 90),
# 	'forward_low_range': (0, 0),
# 	'forward_hi_range': (40, 800),
# 	'forward_p_detect': (0.2, 1),
# 	'current-heading': (0, 359),
# 	'drift-speed': (0, 1),
# }

# nav_variables = {
# 	'turn-radius': (2, 20),
# 	'uuv-speed': (2, 4),
# 	'nav-bearing-std': (0, 3),
# 	'nav-velocity-std-cm': (0, 4),
# 	'side_angle': (20, 120),
# 	'side_hi_range': (200, 1000),
# 	'side_p_detect': (0.5, 1),
# 	'forward_angle': (20, 90),
# 	'forward_hi_range': (20, 800),
# 	'forward_p_detect': (0.5, 1),
# }

design = lhs_of_variables(v2_variables, 300, 'corr', 2222)  # use pyDOE's corr optimization

design.to_csv(output_file, sep=' ', header=False, index=False)
design.to_csv('../experiments/version2/v2_pyDOE_lhs_description.csv')
