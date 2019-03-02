import pyDOE2
import pandas as pd
import xml.etree.ElementTree as ET


def lhs_of_variables(variables_dict, number_of_samples, method='m', random_seed=None):
	# generate a dict with the enumerated values to run for each variable in a latin hypercube experiment
	num_factors = len(variables_dict)  # get the number of factors for the experiment

	design = pd.DataFrame(pyDOE2.lhs(num_factors, samples=number_of_samples, criterion=method, random_state=random_seed), columns=variables_dict.keys())

	for var in variables_dict:
		max_var_value = variables_dict[var][1]
		min_var_value = variables_dict[var][0]
		design[var] = design[var].apply(lambda x: (max_var_value-min_var_value)*x + min_var_value)

	return design


def generate_experiment(ex_name, ex_reps, run_metrics_on_steps,
						ex_values,
						commands={'go commands': 'go', 'setup commands': 'setup', 'metrics': 'count turtles'},
						**kwargs):
	"""
	Create formatted XML to run an experiment in NetLogo BehaviorSpace.
	:param ex_name: The "Experiment Name" that will be specified to call this setup.  It must be a string
	:param ex_reps: The number of times each run should be repeated.  It must be an integer
	:param run_metrics_on_steps:  Whether metrics are measured each step or at the end.  String "true" or "false"
	:param ex_values:  a dictionary with keys as the variable names and values as lists of each value for the experiment
	:param commands:  A dictionary with keys "setup commands", "go commands", and "metrics".  Values are netlogo command strings
	:param kwargs:  Optional setup values: "time_limit" "exit_condition"
	:return: an xml formatted string
	"""
	# create the xml to run an experiment
	# always use enumeratedValueSet because the stepping is calculated by this library
	experiment = ET.Element("experiment", {'name': str(ex_name), 'repetitions': str(ex_reps), 'runMetricsEveryStep': str(run_metrics_on_steps)})

	setup = ET.SubElement(experiment, 'setup')
	setup.text = commands["setup commands"]

	go = ET.SubElement(experiment, 'go')
	go.text = commands["go commands"]

	metrics = ET.SubElement(experiment, 'metric')
	metrics.text = commands["metrics"]

	if 'time_limit' in kwargs:
		ET.SubElement(experiment, 'timeLimit', {'steps': kwargs.get('time_limit')})

	if 'exit_condition' in kwargs:
		exit_cond = ET.SubElement(experiment, 'exitCondition')
		exit_cond.text = kwargs.get('exit_condition')

	# iterate through the experiment values
	for val in ex_values:
		var_set = ET.SubElement(experiment, 'enumeratedValueSet', {'variable': val})
		for v in ex_values[val]:
			ET.SubElement(var_set, 'value', {'value': str(v)})

	return ET.dump(experiment)
