class CompletePIDController:
    def __init__(self, kp, ki, kd, kff, bff, Ts,
                 I_min=-float('inf'), I_max=float('inf'),
                 deadzone_enabled=True, deadzone_tol=1e-6,
                 u_min=-1.0, u_max=1.0,
                 deriv_filter_coeff=10.0):
        """
        Initializes the incremental wheel speed controller with anti-windup, deadzone,
        output clamping, and tunable derivative filtering.

        Parameters:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            kff (float): Feedforward gain.
            bff (float): Feedforward bias.
            Ts (float): Sampling period (seconds).
            I_min (float): Lower integrator state limit (anti-windup).
            I_max (float): Upper integrator state limit (anti-windup).
            deadzone_enabled (bool): Enable/disable deadzone logic.
            deadzone_tol (float): Tolerance for deadzone (values near zero).
            u_min (float): Minimum controller output.
            u_max (float): Maximum controller output.
            deriv_filter_coeff (float): The filtering coefficient N for the derivative term.
                                        Higher values mean less filtering.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff = kff
        self.bff = bff
        self.Ts = Ts

        self.I_min = I_min
        self.I_max = I_max

        self.deadzone_enabled = deadzone_enabled
        self.deadzone_tol = deadzone_tol

        self.u_min = u_min
        self.u_max = u_max

        # Derivative filter coefficient (N)
        self.deriv_filter_coeff = deriv_filter_coeff
        # Internal state for the filtered derivative
        self.d_filter = 0.0

        # Internal states for incremental PID (velocity form).
        self.I = 0.0         # Integral state.
        self.e_prev = 0.0    # Previous error.
        self.e_prev2 = 0.0   # Error two time-steps ago.
        self.u_pid = 0.0     # Accumulated PID output (without feedforward).

    def change_param(self, kp=None, ki=None, kd=None, kff=None, bff=None,
                     I_min=None, I_max=None, deadzone_enabled=None, deadzone_tol=None,
                     u_min=None, u_max=None, deriv_filter_coeff=None):
        """
        Dynamically change controller parameters.

        Parameters (all optional):
            kp, ki, kd, kff, bff: Gains and biases.
            I_min (float): New lower limit for the integrator.
            I_max (float): New upper limit for the integrator.
            deadzone_enabled (bool): Enable/disable deadzone.
            deadzone_tol (float): New tolerance for the deadzone condition.
            u_min (float): New minimum controller output.
            u_max (float): New maximum controller output.
            deriv_filter_coeff (float): New derivative filter coefficient (N).
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if kff is not None:
            self.kff = kff
        if bff is not None:
            self.bff = bff
        if I_min is not None:
            self.I_min = I_min
        if I_max is not None:
            self.I_max = I_max
        if deadzone_enabled is not None:
            self.deadzone_enabled = deadzone_enabled
        if deadzone_tol is not None:
            self.deadzone_tol = deadzone_tol
        if u_min is not None:
            self.u_min = u_min
        if u_max is not None:
            self.u_max = u_max
        if deriv_filter_coeff is not None:
            self.deriv_filter_coeff = deriv_filter_coeff

    def update(self, setpoint, measured_speed, debug=False, compensate=False):
        """
        Compute the control output (voltage/torque command) based on the desired setpoint
        and measured speed. This method uses an incremental PID (velocity form) with:
          - Proportional incremental: kp*(e[k] - e[k-1])
          - Integral incremental: ki*Ts*e[k] (with anti-windup clamping)
          - Derivative incremental: computed using a tunable first-order filter.
        A dynamic feedforward term (kff*setpoint + bff) is then added.
        Finally, the output is clamped to the interval [u_min, u_max].

        If debug is True, a tuple (u, debug_info) is returned where debug_info is a
        dictionary detailing each component's contribution.

        Parameters:
            setpoint (float): Desired wheel speed.
            measured_speed (float): Current measured wheel speed.
            debug (bool): If True, return detailed debug information.

        Returns:
            u (float): Clamped control output.
            OR if debug is True:
            (u, debug_info): Tuple containing the control output and a dictionary with debug info.
        """
        # Deadzone: if both setpoint and measured speed are effectively zero,
        # reset internal states and return 0.
        if (self.deadzone_enabled and
            abs(setpoint) < self.deadzone_tol and
            abs(measured_speed) < self.deadzone_tol):
            self.reset()
            if debug:
                debug_info = {
                    "deadzone": True,
                    "setpoint": setpoint,
                    "measured_speed": measured_speed
                }
                return 0.0, debug_info
            return 0.0

        # Compute current error.
        error = setpoint - measured_speed

        # Incremental proportional term.
        delta_P = self.kp * (error - self.e_prev)

        # Incremental integral term.
        delta_I = self.ki * self.Ts * error

        # Update the integrator with anti-windup clamping.
        I_new = self.I + delta_I
        I_new = max(self.I_min, min(I_new, self.I_max))
        effective_delta_I = I_new - self.I
        self.I = I_new

        # --- Derivative Term with Tunable Filtering ---
        # First, compute the raw derivative using the two-sample (second order) difference:
        raw_delta_D = self.kd / self.Ts * (error - 2 * self.e_prev + self.e_prev2)
        # Apply a first-order low-pass filter to the derivative increment:
        self.d_filter = (self.deriv_filter_coeff * self.Ts * raw_delta_D + self.d_filter) / (1 + self.deriv_filter_coeff * self.Ts)
        delta_D = self.d_filter
        # --- End Derivative Term ---

        # Total incremental change.
        delta_u = delta_P + effective_delta_I + delta_D

        # Update the accumulated PID output.
        self.u_pid += delta_u

        # Compute dynamic feedforward.
        feedforward = self.kff * setpoint + self.bff

        if compensate:
            feedforward = self.throttle_balance(feedforward)

        # Compute the unsaturated control output.
        u_unsat = self.u_pid + feedforward

        # Clamp the final output.
        u = max(self.u_min, min(u_unsat, self.u_max))

        # Prepare debug information.
        debug_info = {
            "error": error,
            "delta_P": delta_P,
            "delta_I": effective_delta_I,
            "raw_delta_D": raw_delta_D,
            "filtered_delta_D": delta_D,
            "u_pid": self.u_pid,
            "feedforward": feedforward,
            "u_unsat": u_unsat,
            "u_clamped": u,
            "I_state": self.I,
            "e_prev": self.e_prev,
            "e_prev2": self.e_prev2,
            "deadzone": False,
            "deriv_filter_coeff": self.deriv_filter_coeff
        }

        # Update error history.
        self.e_prev2 = self.e_prev
        self.e_prev = error

        if debug:
            return u, debug_info
        return u

    def reset(self):
        """
        Reset the controller's internal states (integrator, derivative filter, and error history).
        """
        self.I = 0.0
        self.e_prev = 0.0
        self.e_prev2 = 0.0
        self.u_pid = 0.0
        self.d_filter = 0.0

    def throttle_balance(self,throttle_reversed:float) -> float:

        # pick the one need compensation
        self.compk = 2.83 
        self.compb = 1.94
        
        return (1 + (self.compk*throttle_reversed + self.compb)/100) * throttle_reversed