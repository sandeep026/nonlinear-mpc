from typing import Literal

import casadi as cs
import numpy as np
from pydantic import (
    BaseModel,
    ConfigDict,
    PositiveFloat,
    PositiveInt,
    field_validator,
    model_validator,
)

dis_methods = Literal["fw_euler", "bw_euler", "trap", "her_sim"]


class Nmpc(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    nx: PositiveInt
    nu: PositiveInt
    ts: PositiveFloat
    n: PositiveInt
    Q: cs.DM
    Qend: cs.DM
    R: cs.DM
    f: cs.Function
    method: dis_methods
    x_lb: cs.DM
    x_ub: cs.DM
    u_lb: cs.DM
    u_ub: cs.DM

    @field_validator("Q", "Qend")
    @classmethod
    def check_psd(cls, mat):
        matnp = mat.full()
        if not np.allclose(matnp, matnp.T, atol=1e-10, rtol=0):
            raise ValueError("Q or Qend must be symmetric")

        if np.min(np.linalg.eigvalsh(matnp)) < -1e-10:
            raise ValueError("Q or Qend must be positive semidefinite")

        return mat

    @field_validator("R")
    @classmethod
    def check_pd(cls, mat):
        matnp = mat.full()
        if not np.allclose(matnp, matnp.T, atol=1e-10, rtol=0):
            raise ValueError("R must be symmetric")

        if np.min(np.linalg.eigvalsh(matnp)) <= 1e-10:
            raise ValueError("R must be positive definite")

        return mat

    @field_validator("f")
    @classmethod
    def func_size(cls, f):
        if f.n_in() != 3:
            raise ValueError(
                f"Expected (t,x,u) for dynamics as input but received ({f.n_in()}) inputs"
            )

        if f.n_out() != 1:
            raise ValueError(
                f"Expected a column vector for dynamics but received multiple vectors ({f.n_out()})"
            )
        return f

    @model_validator(mode="after")
    def validate_mpc(self):

        nx, nu = self.nx, self.nu

        shapes = {
            "Q": (nx, nx),
            "R": (nu, nu),
            "Qend": (nx, nx),
            "x_lb": (nx, 1),
            "x_ub": (nx, 1),
            "u_lb": (nu, 1),
            "u_ub": (nu, 1),
        }

        for name, shape in shapes.items():
            value = getattr(self, name)
            if value.shape != shape:
                raise ValueError(f"{name}: expected {shape}, got {value.shape}")

        inputs = ["t", "x", "u"]
        for i, j in enumerate([(1, 1), (nx, 1), (nu, 1)]):
            if self.f.size_in(i) != j:
                raise ValueError(
                    f"Dimension of {inputs[i]} must be {j} but received{self.f.size_in(i)}"
                )

        if self.f.size_out(0) != (nx, 1):
            raise ValueError(
                f"dynamics vector field must be shape {(nx, 1)} but received {self.f.size_out(0)}"
            )

        if np.any(self.x_lb.full() > self.x_ub.full()):
            raise ValueError("lower must be <= upper bound for x")

        if np.any(self.u_lb.full() > self.u_ub.full()):
            raise ValueError("lower must be <= upper bound for u")

        return self

    def generate_NLP(self):

        nlp = cs.Opti()

        n = self.n
        nx, nu = self.nx, self.nu
        ts = self.ts
        t0 = nlp.parameter(1)
        t = cs.linspace(t0, t0 + n * ts, n + 1).T

        X = nlp.variable(nx, n + 1)
        U = nlp.variable(nu, n)

        # time
        x0 = nlp.parameter(nx, 1)
        Xref = nlp.parameter(nx, n + 1)
        Uref = nlp.parameter(nu, n)

        # decision vector
        W = cs.vcat([cs.vec(X), cs.vec(U)])

        # objective
        Q = self.Q
        Qend = self.Qend
        R = self.R

        H = cs.diagcat(
            cs.kron(cs.DM.eye(n), Q * ts), Qend, cs.kron(cs.DM.eye(n), R * ts)
        )
        dW = cs.vcat([cs.vec(X - Xref), cs.vec(U - Uref)])
        objective = 0.5 * dW.T @ H @ dW

        # equality constriants
        # initial condition
        initial = X[:, 0] - x0
        # dynamics
        dynamics = self.get_discrete_dynamics(t, X, U)
        G = cs.vcat([initial, cs.vec(dynamics)])

        # bounds
        X_lb = cs.repmat(self.x_lb, 1, n + 1)
        X_ub = cs.repmat(self.x_ub, 1, n + 1)
        U_lb = cs.repmat(self.u_lb, 1, n)
        U_ub = cs.repmat(self.u_ub, 1, n)
        W_lb = cs.vcat([cs.vec(X_lb), cs.vec(U_lb)])
        W_ub = cs.vcat([cs.vec(X_ub), cs.vec(U_ub)])

        nlp.minimize(objective)
        nlp.subject_to(G == 0)
        nlp.subject_to(nlp.bounded(W_lb, W, W_ub))

        nmpc = {}
        nmpc["nlp"] = nlp
        nmpc["X"] = X
        nmpc["U"] = U
        nmpc["t0"] = t0
        nmpc["x0"] = x0
        nmpc["Xref"] = Xref
        nmpc["Uref"] = Uref

        return nmpc

    def get_discrete_dynamics(self, t, X, U) -> cs.MX:
        F = self.f.map(self.n, "serial")
        Xl = X[:, 0:-1]
        Xr = X[:, 1:]
        tl = t[:, 0:-1]
        tr = t[:, 1:]
        h = self.ts
        dX = Xr - Xl
        Fl = F(tl, Xl, U)
        Fr = F(tr, Xr, U)
        if self.method == "bw_euler":
            defect = dX - Fr * h
            return defect
        elif self.method == "fw_euler":
            defect = dX - Fl * h
            return defect
        elif self.method == "trap":
            defect = dX - 0.5 * h * (Fl + Fr)
            return defect
        elif self.method == "her_sim":
            Xm = 0.5 * (Xr + Xl) + h / 8 * (Fl - Fr)
            tm = 0.5 * (tr + tl)
            Fm = F(tm, Xm, U)
            defect = dX - h / 6 * (Fl + 4 * Fm + Fr)
            return defect
        else:
            raise ValueError("Wrong option for method")


hint_1 = None | dict


def create_nmpc_func(
    nmpc: dict, solver: str = "ipopt", opts: hint_1 = None
) -> cs.Function:

    if opts is None:
        opts = {}
    nmpc["nlp"].solver(solver, opts)
    primals = [nmpc["X"], nmpc["U"]]
    primal_lab = ["X", "U"]
    duals = [nmpc["nlp"].lam_g]
    duals_lab = ["lam"]
    parameters = [nmpc["t0"], nmpc["x0"], nmpc["Xref"], nmpc["Uref"]]
    parameters_lab = ["t0", "x0", "Xref", "Uref"]
    inputs = primals + duals + parameters
    inputs_lab = primal_lab + duals_lab + parameters_lab
    outputs = primals + duals
    outputs_lab = primal_lab + duals_lab
    nmpc_fun = nmpc["nlp"].to_function(
        "nmpc_fun",
        inputs,
        outputs,
        inputs_lab,
        outputs_lab,
        {"allow_duplicate_io_names": True},
    )

    return nmpc_fun
