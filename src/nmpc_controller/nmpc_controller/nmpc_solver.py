import math
import numpy as np
import casadi as ca


class NMPCSolver:
    """
    Unicycle NMPC solver using CasADi Opti interface.

    State  s = [x, y, theta]
    Input  u = [v, w]
    Model:
        dx     = v * cos(theta)
        dy     = v * sin(theta)
        dtheta = w

    Integration: RK4 with step dt
    Horizon: N steps

    Cost:
        sum_{k=0}^{N-1} ( s_err_k' Q s_err_k + u_k' R u_k )
        + s_err_N' Q_N s_err_N   (terminal cost)

    The NLP is built once in the constructor; subsequent calls to solve()
    only update parameter values, so the symbolic graph is reused.
    """

    def __init__(
        self,
        N: int = 20,
        dt: float = 0.1,
        Q: list = None,
        R: list = None,
        Q_N: list = None,
        v_max: float = 0.3,
        w_max: float = 1.5,
    ):
        self.N = N
        self.dt = dt
        self.v_max = v_max
        self.w_max = w_max

        self.Q   = np.diag(Q   or [2.0, 2.0, 0.5])
        self.R   = np.diag(R   or [0.1, 0.05])
        self.Q_N = np.diag(Q_N or [4.0, 4.0, 1.0])

        self._build_problem()

        # Previous solution for warm starting
        self._prev_u = np.zeros((N, 2))    # (N, 2)
        self._prev_s = None                # (N+1, 3), None on first call

    # ------------------------------------------------------------------
    def _rk4_step(self, s, u):
        """One RK4 integration step using CasADi symbolic variables."""
        def f(s_, u_):
            return ca.vertcat(
                u_[0] * ca.cos(s_[2]),
                u_[0] * ca.sin(s_[2]),
                u_[1],
            )

        k1 = f(s, u)
        k2 = f(s + self.dt / 2 * k1, u)
        k3 = f(s + self.dt / 2 * k2, u)
        k4 = f(s + self.dt * k3, u)
        return s + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    # ------------------------------------------------------------------
    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to [-pi, pi] inside CasADi symbolic graph."""
        pi = math.pi
        return ca.fmod(angle + pi, 2 * pi) - pi

    # ------------------------------------------------------------------
    def _build_problem(self):
        """
        Build the Opti NLP once.
        Decision variables S (states) and U (inputs) are declared here.
        Current state and reference trajectory are declared as parameters
        so that only set_value() calls are needed at each solve step.
        """
        opti = ca.Opti()
        N = self.N

        S = opti.variable(3, N + 1)   # state trajectory
        U = opti.variable(2, N)       # input trajectory

        s0_param    = opti.parameter(3)          # current state
        s_ref_param = opti.parameter(3, N + 1)   # reference segment

        # Cost
        cost = 0
        for k in range(N):
            err = S[:, k] - s_ref_param[:, k]
            err_wrapped = ca.vertcat(err[0], err[1], self._wrap_angle(err[2]))
            cost += err_wrapped.T @ self.Q @ err_wrapped
            cost += U[:, k].T @ self.R @ U[:, k]

        err_N = S[:, N] - s_ref_param[:, N]
        err_N_wrapped = ca.vertcat(err_N[0], err_N[1], self._wrap_angle(err_N[2]))
        cost += err_N_wrapped.T @ self.Q_N @ err_N_wrapped
        opti.minimize(cost)

        # Dynamics constraints
        opti.subject_to(S[:, 0] == s0_param)
        for k in range(N):
            opti.subject_to(S[:, k + 1] == self._rk4_step(S[:, k], U[:, k]))

        # Input box constraints
        opti.subject_to(opti.bounded(0.0, U[0, :], self.v_max))   # v >= 0: forward-only
        opti.subject_to(opti.bounded(-self.w_max, U[1, :], self.w_max))

        # IPOPT options — suppress stdout, limit iterations for real-time use
        opti.solver('ipopt', {
            'ipopt.print_level': 0,
            'ipopt.max_iter': 200,
            'ipopt.tol': 1e-4,
            'ipopt.acceptable_tol': 1e-3,
            'ipopt.warm_start_init_point': 'yes',
            'print_time': 0,
        })

        self._opti        = opti
        self._S           = S
        self._U           = U
        self._s0_param    = s0_param
        self._s_ref_param = s_ref_param

    # ------------------------------------------------------------------
    def solve(
        self,
        current_state: np.ndarray,
        ref_segment: np.ndarray,
    ) -> tuple:
        """
        Solve the NMPC problem for one step.

        Args:
            current_state: shape (3,)  — [x, y, theta]
            ref_segment:   shape (N+1, 3) — reference states for horizon

        Returns:
            u_opt   (N, 2)   — optimal input sequence [v, w]
            s_opt   (N+1, 3) — predicted state trajectory
            success bool     — True if IPOPT solved successfully
        """
        opti = self._opti

        opti.set_value(self._s0_param,    current_state)
        opti.set_value(self._s_ref_param, ref_segment.T)  # (3, N+1)

        # Warm start: reuse previous solution as initial guess
        if self._prev_s is None:
            s_init = np.tile(current_state, (self.N + 1, 1)).T
            opti.set_initial(self._S, s_init)
        else:
            opti.set_initial(self._S, self._prev_s.T)   # (3, N+1)

        opti.set_initial(self._U, self._prev_u.T)       # (2, N)

        try:
            sol = opti.solve()
            u_opt = sol.value(self._U).T    # (N, 2)
            s_opt = sol.value(self._S).T    # (N+1, 3)
            self._prev_u = u_opt
            self._prev_s = s_opt
            return u_opt, s_opt, True

        except Exception:
            # Solver failed — return previous solution as fallback
            return self._prev_u, (self._prev_s if self._prev_s is not None
                                  else np.zeros((self.N + 1, 3))), False
