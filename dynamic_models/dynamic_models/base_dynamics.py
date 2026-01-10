from casadi import SX, vertcat, integrator

class BaseDynamics():
    '''
    Base class for the system dynamics. Defines the basic structure of the system dynamics.
    Defines an integrator using casadi.
    Specific parameters are defined in the child classes.
    '''
    def __init__(self):
           pass
           
    def define_system_variables(self, variable_names):
            """
            Define the casadi variables for the system
            """

            casadi_variable = vertcat(*[SX.sym(name) for name in variable_names]) 

            return casadi_variable


    def create_system_integrator(self, x, u, dxdt, dt):
            """
            Create a casadi integrator
            """
            dae = {'x': x, 'p': u, 'ode': dxdt}
            opts = {'tf': dt}

            I = integrator('I', 'idas', dae, opts)
            return I
    

    def create_system_integrator_plus_human(self, x, u, u_h, dxdt, dt):
        """
        Cooperative system dynamics.
        Create a casadi integrator with second input from human
        """
        dae = {'x': x, 'p': vertcat(u, u_h), 'ode': dxdt}
        opts = {'tf': dt}

        I = integrator('I', 'idas', dae, opts)
        return I