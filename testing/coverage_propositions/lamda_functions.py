# File to construct lambda functions based on user input
def construct_lambda_functions(sys_prop_list, env_prop_list, connective):
    func = []
    if env_prop_list == [] and sys_prop_list != []:
        func = lambda ns, ne: ns in sys_prop_list
    if sys_prop_list == [] and env_prop_list != []:
        func = lambda ns, ne: ne in env_prop_list
    if sys_prop_list != [] and env_prop_list!= []:
        if connective == "or":
            func = lambda ns, ne: (ns in sys_prop_list or ne in env_prop_list)
        if connective == "and":
            func = lambda ns, ne: (ns in sys_prop_list and ne in env_prop_list)
    return func