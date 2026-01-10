from .double_integrator import DoubleIntegrator
from .mass_damping import MassDamping

def create_dynamic_model(system_type, system_parameters, x_names, u_names):
        """
        Function defining system dynamics based on system type.
        """
        if system_type == 0:
            return DoubleIntegrator(x_names, u_names, system_parameters)
        if system_type == 1:
            return MassDamping(x_names, u_names, system_parameters)
        else:
            raise NotImplementedError("System type not implemented")
        