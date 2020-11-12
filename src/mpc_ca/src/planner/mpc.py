import casadi
import numpy as np
from scipy.spatial import ConvexHull

from planner.ros_interface import Logger as logging
from planner.utils import interpolate_path


class NonLinearMPC(object):
    """NonLinearMPC for path generation, which computes the fastest feasible
    path given obstacle constraints, an initial state and a goal state.
    To do this, it solves an optimization problem.

    :param planner_parameters: Parameters required for the NonLinearMPC
    :type planner_parameters: dictionary
    """

    def __init__(self, planner_parameters):
        ## Prediction horizon (in steps)
        self.N = planner_parameters['N']
        ## Car wheelbase (in m)
        self.wheelbase = planner_parameters['wheelbase']
        ## Car width (in m)
        self.width = planner_parameters['width']
        ## Car length (in m)
        self.length = planner_parameters['length']
        ## Car model as a rectangle
        origin = np.array([0, 0, 0])
        edge_points = self._get_car_edges_from_rearwheel(origin)
        A, b = self._compute_convex_hull_equations(edge_points)
        self.G = casadi.DM(A)
        self.g = casadi.DM(b)
        ## Weights on the inputs (speed, angular_speed) for all horizon steps
        self.R = casadi.DM([[planner_parameters['R_speed'], 0],
                            [0, planner_parameters['R_steering_angle']]])
        ## Weight on the total time
        self.kappa = planner_parameters['kappa']
        ## Number of states (x, y angle)
        self.nx = 3
        ## Number of inputs (speed, angular_speed)
        self.nu = 2
        ## Minimum speed (in meters/seconds)
        self.speed_min = planner_parameters['speed_min']
        ## Maximum speed (in meters/seconds)
        self.speed_max = planner_parameters['speed_max']
        ## Minimum steering angle (in radians)
        self.steering_angle_min = planner_parameters['steering_angle_min']
        ## Maximum steering angle (in radians)
        self.steering_angle_max = planner_parameters['steering_angle_max']
        ## Minimum distance error to consider that the planner has reached the goal (in meters)
        self.distance_tolerance = planner_parameters['distance_tolerance']
        ## Minimum angle error to consider that the planner has reached the goal (in radians)
        self.angle_tolerance = planner_parameters['angle_tolerance']
        ## Casadi solver
        self.solver = casadi.Opti()
        ## Casadi states X = [x, y, yaw]
        self.X = None
        ## Casadi inputs U = [speed, steering_angle]
        self.U = None
        ## Casadi total time T
        self.T = None
        ## Casadi initial state X_0 = [x_0, y_0, yaw_0]
        self.initial_state = None
        ## Casadi goal state X_G = [x_G, y_G, yaw_G]
        self.goal_state = None
        ## Casadi list of lambda variables (obstacle avoidance)
        self.lambdas = []
        ## Casadi list of mu variables (obstacle avoidance)
        self.mus = []
        ## Casadi list of a variables (obstacle as convex hull)
        self.as_ = []
        ## Casadi list of b variables (obstacle as convex hull)
        self.bs = []

    def setup_optimization_problem(self, obstacles):
        """Sets up the optimization problem, defining symbolic variables and
        adding all of the following:
            - Equality constraints
            - Inequality constraints
            - Cost function
        Which will then be replaced in terms of the actual inputs and solved,
        in another method

        :param obstacle_edges: List of edges with coordinates [x, y]
        :type obstacle_edges: list
        """
        self.X = self.solver.variable(self.nx, self.N + 2)
        self.U = self.solver.variable(self.nu, self.N + 1)
        self.T = self.solver.variable()

        for obstacle_edges in obstacles:
            self.lambdas.append(self.solver.variable(4, self.N + 1))
            self.mus.append(self.solver.variable(4, self.N + 1))

            A, b = self._compute_convex_hull_equations(obstacle_edges)

            self.as_.append(casadi.DM(A))
            self.bs.append(casadi.DM(b))

        self.initial_state = self.solver.parameter(self.nx)
        self.goal_state = self.solver.parameter(self.nx)

        self._add_equality_constraints()
        self._add_inequality_constraints()
        self._add_cost_function()

        self.solver.solver('ipopt')

    def _add_equality_constraints(self):
        """Adds the equality constraints to the solver
        """
        self.solver.subject_to(self.X[:, 0] == self.initial_state)

        dt = self.T / self.N

        d_yaw = casadi.MX.sym('d_yaw')

        rot = casadi.Function(
            'rot', [d_yaw],
            [casadi.cos(d_yaw), casadi.sin(d_yaw),
             -casadi.sin(d_yaw), casadi.cos(d_yaw)]
        )

        for k in range(self.N + 1):
            self.solver.subject_to(
                [self.X[0, k + 1] == self.X[0, k] + dt * self.U
                 [0, k] * casadi.cos(self.X[2, k]),
                 self.X[1, k + 1] == self.X[1, k] + dt * self.U
                 [0, k] * casadi.sin(self.X[2, k]),
                 self.X[2, k + 1] == self.X[2, k] + dt * casadi.tan(self.U[1, k]) / self.wheelbase
                 ]
            )

            rotation = rot(self.X[2, k])
            rotation = casadi.reshape(casadi.vertcat(*rotation), 2, 2)

            for ind in range(len(self.lambdas)):
                self.solver.subject_to(
                    casadi.mtimes(self.G.T, self.mus[ind][:, k]) + casadi.mtimes(
                        casadi.mtimes(rotation.T, self.as_[ind].T),
                        self.lambdas[ind][:, k]) == 0)

                result = casadi.mtimes(self.as_[ind].T, self.lambdas[ind][:, k])

                self.solver.subject_to(result[0] ** 2 + result[1] ** 2 == 1)

    def _add_inequality_constraints(self):
        """Adds the inequality constraints to the solver
        """
        self.solver.subject_to(
            casadi.norm_2(self.goal_state[0:2] - self.X[0:2, -1])
            <= self.distance_tolerance
        )

        self.solver.subject_to(
            casadi.fabs(self.goal_state[2] - self.X[2, -1]) <= self.angle_tolerance
        )

        self.solver.subject_to(self.T >= 0)

        self.solver.subject_to(self.speed_min <= self.U[0, :])
        self.solver.subject_to(self.U[0, :] <= self.speed_max)

        self.solver.subject_to(self.steering_angle_min <= self.U[1, :])
        self.solver.subject_to(self.U[1, :] <= self.steering_angle_max)

        dx = casadi.MX.sym('dx')
        dy = casadi.MX.sym('dy')

        trans = casadi.Function('trans', [dx, dy], [dx, dy])

        for k in range(self.N + 1):
            translation = trans(self.X[0, k], self.X[1, k])
            translation = casadi.vertcat(*translation)

            for ind in range(len(self.lambdas)):
                self.solver.subject_to(
                    casadi.mtimes(-self.g.T, self.mus[ind][:, k])
                    + casadi.mtimes(
                        (casadi.mtimes(self.as_[ind], translation) - self.bs[ind]).T,
                        self.lambdas[ind][:, k]) > 0
                )

        for ind in range(len(self.lambdas)):
            self.solver.subject_to(casadi.vec(self.lambdas[ind]) >= 0)
            self.solver.subject_to(casadi.vec(self.mus[ind]) >= 0)

    def _add_cost_function(self):
        """Adds the cost function to the solver
        """
        sum_input_costs = 0

        for k in range(self.N + 1):
            input_cost = casadi.mtimes(self.U[:, k].T, self.R)
            input_cost = casadi.mtimes(input_cost, self.U[:, k])
            sum_input_costs += input_cost

        self.solver.minimize(self.kappa * self.T + sum_input_costs)

    def compute_path(self, initial_state, goal_state, initial_guess=None):
        """Given a initial state and a goal state, it uses the previously
        setup solver to compute the optimal trajectory X in the optimization
        problem.

        :param initial_state: initial pose [x, y, yaw]
        :type initial_state: numpy array
        :param goal_state: goal pose [x, y, yaw]
        :type goal_state: numpy array
        :param initial_guess: initial guess of the path, where each pose
                              is of the format [x, y, yaw], defaults to None
        :type initial_guess: list of lists, optional
        :return: path, where each pose is of the format [x, y, yaw]
        :rtype: numpy array
        """
        self.solver.set_value(self.initial_state, initial_state)
        self.solver.set_value(self.goal_state, goal_state)

        if initial_guess:
            initial_guess = interpolate_path(initial_guess, self.N + 2)
            self.solver.set_initial(self.X, casadi.DM(initial_guess.T))
        else:
            logging.warning(
                'Invalid initial guess! Will do the optimization without it'
            )

        try:
            result = self.solver.solve()
        except RuntimeError as e:
            logging.error(e)
            return None

        trajectory = result.value(self.X)

        return np.array(trajectory[0:2, :].T)

    def _get_car_edges_from_rearwheel(self, rearwheel_center):
        """Given the point in the center of the rear wheel, it computes the
        car edges modeled as a rectangle in the following order:
        (front right, front left, rear left, rear right)

        To simplify computations, three vectors are defined:
        - u_front: Vector from the center of the rear wheel to the front of the car
        - u_rear: Vector from the center of the rear wheel to the rear of the car
        - v: Vector from the center of the rear wheel to the left of the car

        :param rearwheel_center: pose [x, y, yaw]
        :type rearwheel_center: numpy array
        :return: Car edges: (front right, front left, rear left, rear right)
        :rtype: 2-D numpy array
        """
        _, _, yaw_c = rearwheel_center

        u_front = np.array([
            ((self.length + self.wheelbase) / 2) * np.cos(yaw_c),
            ((self.length + self.wheelbase) / 2) * np.sin(yaw_c)
        ])

        u_rear = np.array([
            (-(self.length - self.wheelbase) / 2) * np.cos(yaw_c),
            (-(self.length - self.wheelbase) / 2) * np.sin(yaw_c)
        ])

        v = np.array([
            (-self.width / 2) * np.sin(yaw_c),
            (self.width / 2) * np.cos(yaw_c)
        ])

        center = rearwheel_center[0:2]

        front_right_edge = center + u_front - v
        front_left_edge = center + u_front + v
        rear_left_edge = center + u_rear + v
        rear_right_edge = center + u_rear - v

        return np.array([
            front_right_edge, front_left_edge,
            rear_left_edge, rear_right_edge
        ])

    @staticmethod
    def _compute_convex_hull_equations(edge_points):
        """Given the edge points of a polyhedron, it computes the convex hull
        equations (inequalities that satisfy the relation Ax <= b)

        It uses the library qhull. Important detail from its documentation:
        The normals point outward, i.e., the convex hull satisfies Ax <= -b
        where A is the matrix of coefficients and b is the vector of offsets.

        :param edge_points: list of points (x,y)
        :type edge_points: 2-D numpy array
        :return: A, b matrices which satisfy the relation Ax <= b
        :rtype: tuple of numpy array
        """
        hull = ConvexHull(edge_points)
        equations = hull.equations

        # Negate b to use the conventional representation Ax <= b
        A = equations[:, 0:2]
        b = -equations[:, 2]

        return A, b
