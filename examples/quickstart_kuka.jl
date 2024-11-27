# Set up problem using TrajectoryOptimization.jl and RobotZoo.jl
using TrajectoryOptimization
using Altro
import RobotZoo
using StaticArrays, LinearAlgebra
using RobotDynamics
using Plots
urdf_kuka = joinpath(urdf_folder, "temp","kuka.urdf")
const RD = RobotDynamics

# Use the Cartpole model from RobotZoo
model = Model(urdf_kuka)
n,m = RD.dims(model)
print("n: ", n)
print("\n") 
print("m: ", m)

# # Define model discretization
# N = 101
# tf = 5.
# dt = tf/(N-1)

# # Define initial and final conditions
# x0 = @SVector zeros(n)
# xf = @SVector [3, 0]

# # Set up
# Q = 1.0e-2*Diagonal(@SVector ones(n)) * dt
# Qf = 100.0*Diagonal(@SVector ones(n))
# R = 1.0e-1*Diagonal(@SVector ones(m)) * dt
# obj = LQRObjective(Q,R,Qf,xf,N)

# # Add constraints
# conSet = ConstraintList(n,m,N)
# u_bnd = 300.0
# bnd = BoundConstraint(n,m, u_min=-u_bnd, u_max=u_bnd)
# goal = GoalConstraint(xf)
# add_constraint!(conSet, bnd, 1:N-1)
# add_constraint!(conSet, goal, N)

# # Initialization
# u0 = @SVector fill(0.01,m)
# U0 = [u0 for k = 1:N-1]

# # Define problem
# prob = Problem(model, obj, x0, tf, xf=xf, constraints=conSet)
# # initial_controls!(prob, U0)

# # Solve with ALTRO
# opts = SolverOptions(
#     cost_tolerance_intermediate=1e-2,
#     penalty_scaling=10.,
#     penalty_initial=1.0
# )
# altro = ALTROSolver(prob, opts)
# solve!(altro)

# # # Get some info on the solve
# max_violation(altro)  # 5.896e-7
# cost(altro)           # 1.539
# iterations(altro)     # 44

# # Extract the solution
# X = states(altro)
# U = controls(altro)
# print("X_end: ", X[end])

# #plot position over time and volecity over time
# plot1 = plot(1:N, [x[1] for x in X], label="Position")
# plot!(1:N, [x[2] for x in X], label="Velocity")

# # on a separate plot, plot control over time
# plot2 = plot(1:N-1, [u[1] for u in U], label="Control")

# display(plot1)
# display(plot2)


# # # Extract the solver statistics
# # stats = Altro.stats(altro)   # alternatively, solver.stats
# # stats.iterations             # 44, equivalent to iterations(solver)
# # stats.iterations_outer       # 4 (number of Augmented Lagrangian iterations)
# # stats.iterations_pn          # 1 (number of projected newton iterations)
# # stats.cost[end]              # terminal cost
# # stats.c_max[end]             # terminal constraint satisfaction
# # stats.gradient[end]          # terminal gradient of the Lagrangian
# # dstats = Dict(stats)         # get the per-iteration stats as a dictionary (can be converted to DataFrame)