import numpy as np

class WheelSpeedController:
    def __init__(self, Ts=0.02,
                 Kp=np.array([0.3, 0.3]),
                 Ki=np.array([1.0, 1.0]),
                 umin=0.0, umax=1.0,
                 eps_ref=0.05,           # rad/s: treat as "stop", zero I
                 deadband=0.05,          # throttle deadband (~5%)
                 kick=0.03):             # static-friction kick when starting
        self.Ts = Ts
        self.Kp = np.asarray(Kp, float)
        self.Ki = np.asarray(Ki, float)
        self.umin, self.umax = float(umin), float(umax)
        self.eps_ref = float(eps_ref)
        self.deadband = float(deadband)
        self.kick = float(kick)
        self.integral = np.zeros(2)
        self.u_prev = np.zeros(2)

    def _apply_deadband(self, u):
        # Map (0..1) -> (deadband..1) to overcome ESC deadband
        return self.deadband + (1.0 - self.deadband) * np.clip(u, 0.0, 1.0)

    def compute(self, w_meas, w_ref):
        w_meas = np.asarray(w_meas, float).reshape(2)
        w_ref  = np.maximum(0.0, np.asarray(w_ref, float).reshape(2))  # no negative refs
        err = w_ref - w_meas

        # per-wheel integral reset near zero speed
        zero_mask = (w_ref < self.eps_ref)
        self.integral[zero_mask] = 0.0

        # raw PI
        u_unsat = self.Kp*err + self.Ki*self.integral
        u_sat = np.clip(u_unsat, self.umin, self.umax)

        # conditional integration (one-quadrant):
        # block I when: at max & err>0 (wants more forward we can't give),
        # or at min(0) & err<0 (wants braking we can't give).
        for i in range(2):
            at_max = (u_sat[i] >= self.umax - 1e-9) and (err[i] > 0)
            at_min = (u_sat[i] <= self.umin + 1e-9) and (err[i] < 0)
            if not (at_max or at_min):
                self.integral[i] += self.Ts * err[i]

        # recompute with updated I, then clip
        u = np.clip(self.Kp*err + self.Ki*self.integral, self.umin, self.umax)

        # friction kick only when starting from (near) zero with positive ref
        start_mask = (self.u_prev < 1e-3) & (w_ref >= self.eps_ref)
        u[start_mask] = np.minimum(1.0, u[start_mask] + self.kick)

        # apply ESC deadband mapping
        u_cmd = self._apply_deadband(u)

        self.u_prev = u.copy()
        return u_cmd

    def update_gains(self, kp_left, ki_left, kp_right, ki_right):
        self.Kp = np.array([kp_left,  kp_right], float)
        self.Ki = np.array([ki_left,  ki_right], float)
