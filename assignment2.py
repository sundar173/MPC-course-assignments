import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 10
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference2 = [10, 10, 0]
        self.reference1 = [10, 2, 3.14/2]

    def plant_model(self,  prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        a_t = pedal
        delta = steering

        x_t += v_t*np.cos(psi_t)*dt
        y_t += v_t*np.sin(psi_t)*dt
        psi_t += (v_t/2.5)*np.tan(delta)*dt
        v_t += a_t*dt - v_t/25
        return [x_t, y_t, psi_t, v_t]

    def cost_function(self, u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        for i in range(self.horizon):
            state = self.plant_model(state, self.dt, u[i*2], u[i*2+1])
            cost += (state[0] - ref[0])**2 + (state[1] - ref[1])**2 + (state[2] - ref[2])**2

        return cost

sim_run(options, ModelPredictiveControl)
