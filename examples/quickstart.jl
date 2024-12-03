# Set up problem using TrajectoryOptimization.jl and RobotZoo.jl
using TrajectoryOptimization
using Altro
import RobotZoo.Cartpole
using StaticArrays, LinearAlgebra
using RobotDynamics
using Plots
const RD = RobotDynamics

# Use the Cartpole model from RobotZoo
model = Cartpole()
n,m = RD.dims(model)
println("n: ", n)   
println("m: ", m)

# Define model discretization
N = 41
tf = 2.
dt = tf/(N-1)

# Define initial and final conditions
x0 = @SVector zeros(n)
xf = @SVector [0, pi, 0, 0]  # i.e. swing up

# Set up
Q = 1.0e-5*Diagonal(@SVector ones(n))
Qf = 100.0*Diagonal(@SVector ones(n))
R = 1.0e-1*Diagonal(@SVector ones(m)) * dt
obj = LQRObjective(Q,R,Qf,xf,N)

# Add constraints
conSet = ConstraintList(n,m,N)
u_bnd = 10.0
bnd = BoundConstraint(n,m, u_min=-u_bnd, u_max=u_bnd)
goal = GoalConstraint(xf)
add_constraint!(conSet, bnd, 1:N-1)
add_constraint!(conSet, goal, N)

# Initialization
u0 = @SVector fill(5.0,m)
U0 = [u0 for k = 1:N-1]

# Define problem
prob = Problem(model, obj, x0, tf, xf=xf, constraints=conSet)
initial_controls!(prob, U0)

# Solve with ALTRO
opts = SolverOptions(
    cost_tolerance_intermediate=1e-5,
    penalty_scaling=10.,
    penalty_initial=1.0
)
altro = ALTROSolver(prob, opts)
solve!(altro)

println("max_violation: ", max_violation(altro))
println("cost:          ", cost(altro))
println("iterations:    ", iterations(altro));

# Extract the solution
X = states(altro)
U = controls(altro)

#plot position over time and volecity over time
plot1 = plot(1:N, [x[1] for x in X], label="p")
plot!(1:N, [x[2] for x in X], label="theta")
plot!(1:N, [x[3] for x in X], label="p_dot")
plot!(1:N, [x[4] for x in X], label="theta_dot")

# on a separate plot, plot control over time
plot2 = plot(1:N-1, [u[1] for u in U], label="Control")


# solve with ilqr
ilqr = Altro.iLQRSolver(prob, opts)
initial_controls!(ilqr, U0)
solve!(ilqr)

X = states(ilqr)
U = controls(ilqr)

#plot position over time and volecity over time
plot3 = plot(1:N, [x[1] for x in X], label="p")
plot!(1:N, [x[2] for x in X], label="theta")
plot!(1:N, [x[3] for x in X], label="p_dot")
plot!(1:N, [x[4] for x in X], label="theta_dot")

# on a separate plot, plot control over time
plot4 = plot(1:N-1, [u[1] for u in U], label="Control")

display(plot1)
display(plot2)
display(plot3)
display(plot4)

# using Ipopt
# using MathOptInterface
# const MOI = MathOptInterface

# # Copy problem to avoid modifying the original problem
# # prob_nlp = copy(prob)
# prob_nlp = Problem(model, obj, x0, tf, xf=xf, constraints=conSet)

# # # Add the dynamics and initial conditions as explicit constraints
# TrajectoryOptimization.add_dynamics_constraints!(prob_nlp)

# # Reset our initial guess
# initial_controls!(prob_nlp, U0)
# rollout!(prob_nlp)

# Create the NLP
nlp = TrajOptNLP(prob_nlp, remove_bounds=true, jac_type=:vector);

# optimizer = Ipopt.Optimizer()
# TrajectoryOptimization.build_MOI!(nlp, optimizer)
# MOI.optimize!(optimizer)
# MOI.get(optimizer, MOI.TerminationStatus())

# println("max_violation: ", max_violation(nlp))
# println("cost:          ", cost(nlp));

# using BenchmarkTools

# # Reset initial guess and then benchmark ALTRO solver
# initial_controls!(altro, U0)
# b_altro = benchmark_solve!(altro)

# # Reset initial guess and benchmark Ipopt solver
# initial_controls!(prob_nlp, U0)
# rollout!(prob_nlp)
# nlp = TrajOptNLP(prob_nlp, remove_bounds=true, jac_type=:vector)
# Z0 = copy(TrajectoryOptimization.get_trajectory(nlp).Z)

# optimizer = Ipopt.Optimizer(print_level=0)
# TrajectoryOptimization.build_MOI!(nlp, optimizer)
# Z = MOI.VariableIndex.(1:length(Z0))
# b_ipopt = @benchmark begin
#     MOI.optimize!($optimizer)
#     MOI.set($optimizer, MOI.VariablePrimalStart(), $Z, $Z0)
#     MOI.get($optimizer, MOI.TerminationStatus())
# end

# using Statistics
# @show cost(nlp)
# @show cost(altro)
# @show max_violation(nlp)
# @show max_violation(altro)
# jdg = judge(median(b_altro), median(b_ipopt))
# println("Speed improvement: ", round(1/ratio(jdg).time), "x")
# jdg

# using TrajOptPlots
# using MeshCat
# using Plots

# vis = Visualizer()
# render(vis)

# TrajOptPlots.set_mesh!(vis, model)
# visualize!(vis, altro);
# # visualize!(vis, model, TrajectoryOptimization.get_trajectory(nlp));
# # visualize!(vis, altro, nlp);

# visualize!(vis, model, get_trajectory(altro), get_trajectory(nlp));

# delete!(vis["robot_copies"]);