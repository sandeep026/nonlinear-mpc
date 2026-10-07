import casadi as cs
import matplotlib.pyplot as plt
from nmpc import Nmpc, create_nmpc_func


def closed_loop():

    nx = 2
    nu = 1
    n = 50
    ts = 0.1
    tsym = cs.SX.sym("tsym", 1, 1)
    xsym = cs.SX.sym("xsym", nx, 1)
    usym = cs.SX.sym("usym", nu, 1)
    f = cs.Function(
        "f",
        [tsym, xsym, usym],
        [cs.vertcat(xsym[1], cs.sin(xsym[0]) - 0.1 * xsym[1] + usym * cs.cos(xsym[1]))],
    )

    def simulate(t0, tf, x0, u):
        xnext = x0 + (tf - t0) * f(t0, x0, u)
        return xnext

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

    # need good initial guess if exact hessian is used
    # nmpc_func = create_nmpc_func(
    #    nlp,
    #    solver="sqpmethod",
    #    opts={
    #        "qpsol": "qpoases",
    #        "hessian_approximation": "limited-memory",
    #         'max_iter':1
    #    },
    # )

    warm_start = {
        "ipopt.warm_start_init_point": "yes",
        "ipopt.warm_start_bound_push": 1e-6,
        "ipopt.warm_start_mult_bound_push": 1e-6,
        "ipopt.warm_start_slack_bound_push": 1e-6,
        "ipopt.mu_init": 1e-6,
        "detect_simple_bounds": True,
    }

    nmpc_func = create_nmpc_func(nlp,solver='ipopt', opts=warm_start)

    x0 = cs.DM([cs.pi, 0])
    Xref = cs.DM.zeros(nx, n + 1)
    Uref = cs.DM.zeros(nu, n)
    nsim = 200

    tsim = [0.0]
    Xsim = [x0]
    Usim = []

    # set reference as guess initially
    Xg = cs.vertcat(cs.linspace(cs.pi, 0, n + 1).T, cs.DM.zeros(1, n + 1))
    Ug = cs.linspace(-0.8, 0, n)
    lam_g_g = cs.DM.zeros(nlp["nlp"].lam_g.shape)

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
        mpc_op = nmpc_func(**mpc_ip)
        Xg = cs.horzcat(mpc_op["X"][:, 1:], mpc_op["X"][:, -1])
        Ug = cs.horzcat(mpc_op["U"][:, 1:], mpc_op["U"][:, -1])
        lam_g_g = mpc_op["lam"]
        Usim.append(mpc_op["U"][:, 0])
        xnext = simulate(t0=tsim[-1], tf=tsim[-1] + ts, x0=Xsim[-1], u=Usim[-1])
        Xsim.append(xnext)
        tsim.append(tsim[-1] + ts)

    Xsim = cs.hcat(Xsim).full()
    Usim = cs.hcat(Usim).full()
    tsim = cs.hcat(tsim).full()

    plt.rcParams["figure.dpi"] = 200
    fig, a = plt.subplots(1, 2)
    fig.set_figwidth(10)
    fig.set_figheight(3)
    a[0].plot(tsim.ravel(), Xsim[0, :].ravel(), label="$x_1$")
    a[0].plot(tsim.ravel(), Xsim[1, :].ravel(), label="$x_2$")
    a[0].set_title("State Trajectory")
    a[0].set_ylabel("$x_1, x_2$")
    a[0].set_xlabel("Time (s)")
    a[0].grid(True)
    a[0].legend()
    a[1].step(tsim[:, 0:-1].ravel(), Usim.ravel())
    a[1].set_title("Control Input")
    a[1].set_ylabel("u")
    a[1].set_xlabel("Time (s)")
    a[1].grid(True)
    plt.show()


if __name__ == "__main__":
    closed_loop()
