# Discrete-Time Non-Linear Model Predictive Control (NMPC)

This document provides the complete, mathematically rigorous formulation of the Discrete-Time Non-Linear Model Predictive Control (NMPC) problem as constructed by the `Nmpc` class.

---

## 1. Problem Variables, Parameters, and Dimensions

### Variables and Parameters

| Symbol | Description | Type / Constraint | Dimension |
| :--- | :--- | :--- | :--- |
| $n_x$ | State vector dimension | Integer ($\mathbb{N}_{>0}$) | Scalar |
| $n_u$ | Control input vector dimension | Integer ($\mathbb{N}_{>0}$) | Scalar |
| $N$ | Prediction horizon steps | Integer ($\mathbb{N}_{>0}$) | Scalar |
| $T_s$ | Integration sampling time step | Float ($\mathbb{R}_{>0}$) | Scalar |
| $Q$ | Stage state weighting matrix | Symmetric PSD ($Q = Q^T \succeq 0$) | $\mathbb{R}^{n_x \times n_x}$ |
| $Q_{end}$ | Terminal state weighting matrix | Symmetric PSD ($Q_{end} = Q_{end}^T \succeq 0$) | $\mathbb{R}^{n_x \times n_x}$ |
| $R$ | Control input weighting matrix | Symmetric PD ($R = R^T \succ 0$) | $\mathbb{R}^{n_u \times n_u}$ |
| $x_{lb}, x_{ub}$ | State lower and upper bounds | $x_{lb} \le x_{ub}$ | $\mathbb{R}^{n_x \times 1}$ |
| $u_{lb}, u_{ub}$ | Control lower and upper bounds | $u_{lb} \le u_{ub}$ | $\mathbb{R}^{n_u \times 1}$ |
| $\bar{x}_0$ | Initial state numerical parameter | Parameter | $\mathbb{R}^{n_x \times 1}$ |
| $t_0$ | Initial time parameter | Parameter | $\mathbb{R}$ |
| $x_{ref}$ | state reference parameter | Parameter | $\mathbb{R}^{n_x \times n+1}$ |
| $u_{ref}$ | control reference parameter | Parameter | $\mathbb{R}^{n_x \times n+1}$ |

---

## 2. Matrix and Vector Structures

### Stage Decision and Reference Vectors

* **States:** $x_i \in \mathbb{R}^{n_x \times 1}$ for $i \in \{1, 2, \dots, N+1\}$
* **Controls:** $u_i \in \mathbb{R}^{n_u \times 1}$ for $i \in \{1, 2, \dots, N\}$
* **State References:** $x_{ref, i} \in \mathbb{R}^{n_x \times 1}$ for $i \in \{1, 2, \dots, N+1\}$
* **Control References:** $u_{ref, i} \in \mathbb{R}^{n_u \times 1}$ for $i \in \{1, 2, \dots, N\}$

### Trajectory Matrices (Horizontal Concatenation)

$$X = \begin{bmatrix} x_1 & x_2 & \dots & x_{N+1} \end{bmatrix} \in \mathbb{R}^{n_x \times (N+1)}$$

$$U = \begin{bmatrix} u_1 & u_2 & \dots & u_N \end{bmatrix} \in \mathbb{R}^{n_u \times N}$$

$$X_{ref} = \begin{bmatrix} x_{ref, 1} & x_{ref, 2} & \dots & x_{ref, N+1} \end{bmatrix} \in \mathbb{R}^{n_x \times (N+1)}$$

$$U_{ref} = \begin{bmatrix} u_{ref, 1} & u_{ref, 2} & \dots & u_{ref, N} \end{bmatrix} \in \mathbb{R}^{n_u \times N}$$

### Time Vector

$$t = \begin{bmatrix} t_1 & t_2 & \dots & t_{N+1} \end{bmatrix} \in \mathbb{R}^{1 \times (N+1)}, \quad \text{where } t_i = t_0 + (i-1)T_s$$

### Stacked Optimization Decision Vector ($W$)

Decision matrices are flattened column-wise ($\text{vec}$) and vertically stacked into a single primal vector $W$:

$$\text{vec}(X) = \begin{bmatrix} x_1 \\ x_2 \\ \vdots \\ x_{N+1} \end{bmatrix} \in \mathbb{R}^{n_x(N+1) \times 1}, \quad \text{vec}(U) = \begin{bmatrix} u_1 \\ u_2 \\ \vdots \\ u_N \end{bmatrix} \in \mathbb{R}^{n_u N \times 1}$$

$$W = \begin{bmatrix} \text{vec}(X) \\ \text{vec}(U) \end{bmatrix} \in \mathbb{R}^{(n_x(N+1) + n_u N) \times 1}$$

---

## 3. Optimal Control Transcription & Control Parameterization

### Direct Transcription Framework

The continuous-time optimal control problem (OCP) is converted into a finite-dimensional Non-Linear Program (NLP) using **Direct Transcription** (Simultaneous Method). In this approach, both state trajectory values at discrete nodes $X$ and control trajectory inputs $U$ are optimized simultaneously as decision variables, while dynamic differential equations are discretized into algebraic defect constraints.

### Control Parameterization (Zero-Order Hold)

The control trajectory $u(t)$ is parameterized using a **Piecewise Constant (Zero-Order Hold)** model over each sampling interval:

$$u(t) = u_i, \quad \forall t \in [t_i, t_{i+1}), \quad i \in \{1, 2, \dots, N\}$$

As a result, control decision variables are defined only at stage starts $u_1, u_2, \dots, u_N$, giving $N$ control vectors.

### State Discretization & Collocation Nodes

States are parameterized at $N+1$ discrete temporal grid nodes $t_1, t_2, \dots, t_{N+1}$. Inter-node dynamic continuity is enforced by numerical quadrature schemes generating equality defect constraints $d_i = \mathbf{0}_{n_x \times 1}$ for each step.

---

## 4. Stage-Wise Optimal Control Problem

Given $N$, $T_s$, $t_0$, and initial state $\bar{x}_0$, the NMPC minimizes tracking error over the prediction horizon:

$$
J=\frac{1}{2} \left[ J_{Lagrange} + J_{Mayer}   \right]
$$

$$
 J_{Lagrange}=\sum_{i=1}^{N} \left( (x_i - x_{ref, i})^T (Q \cdot T_s) (x_i - x_{ref, i}) + (u_i - u_{ref, i})^T (R \cdot T_s) (u_i - u_{ref, i}) \right) 
$$

$$
J_{Mayer}=(x_{N+1} - x_{ref, N+1})^T Q_{end} (x_{N+1} - x_{ref, N+1})
$$

subject to following equality constraints and simple bounds on the state
and control.

$$
x_1 = \bar{x}_0
$$

$$
d_i(x_i, x_{i+1}, u_i, t_i, t_{i+1})=0, \quad \forall i \in \{1, 2, \dots, N\}
$$

$$
x_{lb} \le x_i \le x_{ub}, \quad \forall i \in \{1, 2, \dots, N+1\}
$$

$$
u_{lb} \le u_i \le u_{ub}, \quad \forall i \in \{1, 2, \dots, N\}
$$

---

## 5. Discretization Dynamics and Defect Formulations

Let continuous-time dynamics be $\dot{x} = f(t, x, u) \in \mathbb{R}^{n_x \times 1}$. 

Define matrix partitions over stages $i \in \{1, 2, \dots, N\}$:

$$X_l = \begin{bmatrix} x_1 & x_2 & \dots & x_N \end{bmatrix} \in \mathbb{R}^{n_x \times N}, \quad X_r = \begin{bmatrix} x_2 & x_3 & \dots & x_{N+1} \end{bmatrix} \in \mathbb{R}^{n_x \times N}$$

$$t_l = \begin{bmatrix} t_1 & t_2 & \dots & t_N \end{bmatrix} \in \mathbb{R}^{1 \times N}, \quad t_r = \begin{bmatrix} t_2 & t_3 & \dots & t_{N+1} \end{bmatrix} \in \mathbb{R}^{1 \times N}$$

Evaluated dynamics matrices:

$$\mathbf{F}_l = \begin{bmatrix} f(t_1, x_1, u_1) & f(t_2, x_2, u_2) & \dots & f(t_N, x_N, u_N) \end{bmatrix} \in \mathbb{R}^{n_x \times N}$$

$$\mathbf{F}_r = \begin{bmatrix} f(t_2, x_2, u_1) & f(t_3, x_3, u_2) & \dots & f(t_{N+1}, x_{N+1}, u_N) \end{bmatrix} \in \mathbb{R}^{n_x \times N}$$

The defect matrix is defined as

$$
\mathbf{D} = \begin{bmatrix} d_1 & d_2 & \dots & d_N \end{bmatrix} \in \mathbb{R}^{n_x \times N}
$$

for each method is:

### 1. Forward Euler (`fw_euler`)

* **Stage Defect:** $d_i = (x_{i+1} - x_i) - T_s \cdot f(t_i, x_i, u_i)$
* **Defect Matrix:** $\mathbf{D} = (X_r - X_l) - T_s \cdot \mathbf{F}_l$

### 2. Backward Euler (`bw_euler`)

* **Stage Defect:** $d_i = (x_{i+1} - x_i) - T_s \cdot f(t_{i+1}, x_{i+1}, u_i)$
* **Defect Matrix:** $\mathbf{D} = (X_r - X_l) - T_s \cdot \mathbf{F}_r$

### 3. Trapezoidal Method (`trap`)

* **Stage Defect:** $d_i = (x_{i+1} - x_i) - \frac{T_s}{2} \left[ f(t_i, x_i, u_i) + f(t_{i+1}, x_{i+1}, u_i) \right]$
* **Defect Matrix:** $\mathbf{D} = (X_r - X_l) - \frac{T_s}{2} (\mathbf{F}_l + \mathbf{F}_r)$

### 4. Hermite-Simpson Integration (`her_sim`)

Midpoint Matrices are defined as follows,

$$
X_m = \frac{1}{2}(X_r + X_l) + \frac{T_s}{8}(\mathbf{F}_l - \mathbf{F}_r) \in \mathbb{R}^{n_x \times N}
$$
  
$$
t_m = \frac{1}{2}(t_r + t_l) \in \mathbb{R}^{1 \times N}
$$
  
$$
\mathbf{F}_m = \begin{bmatrix} f(t_{m, 1}, x_{m, 1}, u_1) & f(t_{m,2}, x_{m, 2}, u_2) & \dots & f(t_{m, N}, x_{m, N}, u_N) \end{bmatrix} \in \mathbb{R}^{n_x \times N}
$$

* **Stage Defect:** $d_i = (x_{i+1} - x_i) - \frac{T_s}{6} \left[ f(t_i, x_i, u_i) + 4 f(t_{m, i}, x_{m, i}, u_i) + f(t_{i+1}, x_{i+1}, u_i) \right]$
* **Defect Matrix:** $\mathbf{D} = (X_r - X_l) - \frac{T_s}{6} (\mathbf{F}_l + 4 \mathbf{F}_m + \mathbf{F}_r)$

---

## 6. Compact Matrix NLP Formulation

The optimal control problem is cast into a standard Non-Linear Program (NLP):

$$\begin{aligned} 
\min_{W} \quad & J(\Delta W) = \frac{1}{2} \Delta W^T H \Delta W \\ 
\text{s.t.} \quad & G(W) = \mathbf{0} \\ 
& W_{lb} \le W \le W_{ub} 
\end{aligned}$$

### Tracking Error Vector ($\Delta W$)

$$\Delta W = W - W_{ref} = 
\begin{bmatrix} \text{vec}(X - X_{ref}) \\ 
\text{vec}(U - U_{ref}) 
\end{bmatrix} 
\in \mathbb{R}^{(n_x(N+1) + n_u N) \times 1}$$

### Block Hessian Matrix ($H$)

$$
H = \begin{bmatrix}  
I_N \otimes (Q \cdot T_s) & \mathbf{0}_{n_x N \times n_x} & \mathbf{0}_{n_x N \times n_u N} \\  
\mathbf{0}_{n_x \times n_x N} & Q_{end} & \mathbf{0}_{n_x \times n_u N} \\  
\mathbf{0}_{n_u N \times n_x N} & \mathbf{0}_{n_u N \times n_x} & I_N \otimes (R \cdot T_s)  \end{bmatrix} 
\in \mathbb{R}^{(n_x(N+1) + n_u N) \times (n_x(N+1) + n_u N)}
$$

where $\otimes$ denotes the Kronecker product and $I_N$ is the $N \times N$ identity matrix.

### Equality Constraints Vector ($$G(W)$$)

$$
G(W) = \begin{bmatrix} 
x_1 - \bar{x}_0 \\ 
\text{vec}(\mathbf{D}) 
\end{bmatrix} 
= \mathbf{0}
$$

where

$$
\text{vec}(\mathbf{D}) =
\begin{bmatrix}
d_1 \\
d_2 \\
\vdots \\
d_N
\end{bmatrix}
\in \mathbb{R}^{n_x N \times 1}
$$

### Stacked Variable Bounds

Repeated matrix bounds across horizon:

$$
\mathbf{X}_{lb} = 
\begin{bmatrix} x_{lb} & x_{lb} & \dots & x_{lb} \end{bmatrix} \in \mathbb{R}^{n_x \times (N+1)}
$$

$$
\mathbf{X}_{ub} =
\begin{bmatrix} x_{ub} & x_{ub} & \dots & x_{ub} \end{bmatrix} \in \mathbb{R}^{n_x \times (N+1)}
$$

$$\mathbf{U}_{lb} = \begin{bmatrix} u_{lb} & u_{lb} & \dots & u_{lb} \end{bmatrix} \in \mathbb{R}^{n_u \times N}, \quad \mathbf{U}_{ub} = \begin{bmatrix} u_{ub} & u_{ub} & \dots & u_{ub} \end{bmatrix} \in \mathbb{R}^{n_u \times N}$$

Stacked bound vectors:

$$
W_{lb} = 
\begin{bmatrix} 
\text{vec}(\mathbf{X}_{lb}) \\ 
\text{vec}(\mathbf{U}_{lb}) 
\end{bmatrix} \in \mathbb{R}^{(n_x(N+1) + n_u N) \times 1}
$$

$$
W_{ub} =
\begin{bmatrix} 
\text{vec}(\mathbf{X}_{ub}) \\
 \text{vec}(\mathbf{U}_{ub}) 
 \end{bmatrix} 
 \in \mathbb{R}^{(n_x(N+1) + n_u N) \times 1}
 $$

---

## 7. Solver Interface and Mapping (`nmpc_fun`)

The generated NLP solver is wrapped into a callable function with the following input and output structure:

### Inputs

1. `X`: Initial guess for state matrix trajectory $\in \mathbb{R}^{n_x \times (N+1)}$
2. `U`: Initial guess for control matrix trajectory $\in \mathbb{R}^{n_u \times N}$
3. `lam`: Initial guess for equality constraint multipliers $\lambda \in \mathbb{R}^{n_x(N+1) \times 1}$
4. `t0`: Current initial time $t_0 \in \mathbb{R}$
5. `x0`: Current initial state $\bar{x}_0 \in \mathbb{R}^{n_x \times 1}$
6. `Xref`: State reference matrix trajectory $X_{ref} \in \mathbb{R}^{n_x \times (N+1)}$
7. `Uref`: Control reference matrix trajectory $U_{ref} \in \mathbb{R}^{n_u \times N}$

### Outputs

1. `X`: Optimal state matrix trajectory $X^* \in \mathbb{R}^{n_x \times (N+1)}$
2. `U`: Optimal control matrix trajectory $U^* \in \mathbb{R}^{n_u \times N}$
3. `lam`: Optimal equality constraint Lagrange multipliers $\lambda^* \in \mathbb{R}^{n_x(N+1) \times 1}$

## 8. Closed-Loop Simulation Example

The problem to test and validate the code is taken from exercise 5 of the Syscop course on [Model Predictive Control for Renewable Energy Systems](https://www.syscop.de/teaching/ss2023/model-predictive-control-renewable-energy-systems).

To demonstrate the NMPC framework in closed-loop operation, a nonlinear pendulum-like system is stabilized from an inverted equilibrium point 

$$
\bar{x}_0 = 
\begin{bmatrix} \pi & 0 
\end{bmatrix}^T
$$ 

to the origin 

$$
\begin{bmatrix} 0 & 0 
\end{bmatrix}^T
$$

---

### Benchmark System Dynamics

The continuous-time state vector

$$
x(t) =
\begin{bmatrix}
x_1(t) \\
x_2(t)
\end{bmatrix}
\in \mathbb{R}^{2 \times 1}
$$

and control input $u(t) \in \mathbb{R}^{1 \times 1}$ are governed by:

$$
\dot{x}(t)
= f(t, x, u) 
= \begin{bmatrix}
\dot{x}_1 \\
\dot{x}_2
\end{bmatrix}
= \begin{bmatrix} x_2 \\
 \sin(x_1) - 0.1 x_2 + u \cos(x_2)
\end{bmatrix}
 $$

---

### MPC Configuration and Warm-Start Setup

* **Horizon & Sampling:** $N = 50$ steps, $T_s = 0.1\text{ s}$ (Total horizon $N \cdot T_s = 5.0\text{ s}$)
* **Discretization:** Hermite-Simpson integration (`"her_sim"`)
* **Cost Matrices:** $Q = I_2$, $R = 2$, $Q_{end} = 20 I_2$
* **Control Bounds:** $-1.0 \le u_i \le 1.0$
* **Warm-Start Strategy:** At every sampling interval
  1. Ipopt configured for warm start
  2. The optimal primal state and control solutions are shifted left by 1 horizon step, duplicating the final column to form the warm-start guess $(X_g, U_g)$.
  3. The equality Lagrange multipliers $\lambda^*$ are preserved directly as $\lambda_g$.

---

### Receding Horizon Execution Loop

At each simulation time step

1. **Solve NLP:** Compute optimal control using `nmpc_fun`.
2. **Apply Control:** Extract first control input $u_1^*$.
3. **Plant Simulation:** Advance system state
4. **Shift Horizon:** Update guess matrices $(X_g, U_g, \lambda_g)$ for step $k+1$.

---

### Complete Python Implementation

```python
import casadi as cs
import matplotlib.pyplot as plt
from nmpc import Nmpc, create_nmpc_func


def closed_loop():
    # 1. Problem Dimensions & Symbolic Dynamics
    nx = 2
    nu = 1
    n = 50
    ts = 0.1

    tsym = cs.SX.sym("tsym", 1, 1)
    xsym = cs.SX.sym("xsym", nx, 1)
    usym = cs.SX.sym("usym", nu, 1)

    # f(t, x, u)
    f = cs.Function(
        "f",
        [tsym, xsym, usym],
        [cs.vertcat(xsym[1], cs.sin(xsym[0]) - 0.1 * xsym[1] + usym * cs.cos(xsym[1]))],
    )

    # Forward Euler plant simulator
    def simulate(t0, tf, x0, u):
        return x0 + (tf - t0) * f(t0, x0, u)

    # 2. NMPC Object Instantiation
    nmpc = Nmpc(
        nx=nx,
        nu=nu,
        f=f,
        x_lb=cs.DM([-cs.DM.inf()] * 2),
        x_ub=cs.DM([cs.DM.inf()] * 2),
        u_lb=cs.DM(-1),
        u_ub=cs.DM(1),
        method="her_sim",
        ts=ts,
        n=n,
        Q=cs.DM.eye(nx),
        R=cs.DM(2),
        Qend=20 * cs.DM.eye(nx),
    )

    nlp = nmpc.generate_NLP()

    # 3. Solver Setup with Ipopt Warm-Start Options
    warm_start = {
        "ipopt.warm_start_init_point": "yes",
        "ipopt.warm_start_bound_push": 1e-6,
        "ipopt.warm_start_mult_bound_push": 1e-6,
        "ipopt.warm_start_slack_bound_push": 1e-6,
        "ipopt.mu_init": 1e-6,
        "detect_simple_bounds": True,
    }

    nmpc_func = create_nmpc_func(nlp, solver="ipopt", opts=warm_start)

    # 4. Simulation Parameters and Initial Guesses
    x0 = cs.DM([cs.pi, 0])
    Xref = cs.DM.zeros(nx, n + 1)
    Uref = cs.DM.zeros(nu, n)
    nsim = 200

    tsim = [0.0]
    Xsim = [x0]
    Usim = []

    # Initial primal/dual guess trajectories
    Xg = cs.vertcat(cs.linspace(cs.pi, 0, n + 1).T, cs.DM.zeros(1, n + 1))
    Ug = cs.linspace(-0.8, 0, n)
    lam_g_g = cs.DM.zeros(nlp["nlp"].lam_g.shape)

    # 5. Closed-Loop Simulation Loop
    for i in range(nsim):
        mpc_ip = {
            "X": Xg,
            "U": Ug,
            "Xref": Xref,
            "Uref": Uref,
            "t0": tsim[-1],
            "x0": Xsim[-1],
            "lam": lam_g_g,
        }

        # Solve NMPC step
        mpc_op = nmpc_func(**mpc_ip)

        # Shift primal trajectory horizon for next step warm-start
        Xg = cs.horzcat(mpc_op["X"][:, 1:], mpc_op["X"][:, -1])
        Ug = cs.horzcat(mpc_op["U"][:, 1:], mpc_op["U"][:, -1])
        lam_g_g = mpc_op["lam"]

        # Extract first control input and advance plant simulation
        u_apply = mpc_op["U"][:, 0]
        Usim.append(u_apply)

        xnext = simulate(t0=tsim[-1], tf=tsim[-1] + ts, x0=Xsim[-1], u=u_apply)
        Xsim.append(xnext)
        tsim.append(tsim[-1] + ts)

    # Convert logged data to numpy arrays
    Xsim = cs.hcat(Xsim).full()
    Usim = cs.hcat(Usim).full()
    tsim = cs.hcat(tsim).full()

    # 6. Result Plotting
    plt.rcParams["figure.dpi"] = 200
    fig, a = plt.subplots(1, 2, figsize=(10, 3))

    a[0].plot(tsim.ravel(), Xsim[0, :].ravel(), label="$x_1$ (Position)")
    a[0].plot(tsim.ravel(), Xsim[1, :].ravel(), label="$x_2$ (Velocity)")
    a[0].set_title("Closed-Loop State Trajectory")
    a[0].set_ylabel("States ($x_1, x_2$)")
    a[0].set_xlabel("Time (s)")
    a[0].grid(True)
    a[0].legend()

    a[1].step(tsim[:, 0:-1].ravel(), Usim.ravel(), where="post")
    a[1].set_title("Control Input (Zero-Order Hold)")
    a[1].set_ylabel("Control ($u$)")
    a[1].set_xlabel("Time (s)")
    a[1].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    closed_loop()
```    