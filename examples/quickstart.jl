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
Q = 0.5*Diagonal(@SVector ones(n)) * dt
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
display(plot1)
display(plot2)

#save x and u into a csv file
using CSV
using DataFrames
CSV.write("x.csv", DataFrame(X, :auto))
CSV.write("u.csv", DataFrame(U, :auto))


# # solve with ilqr
# ilqr = Altro.iLQRSolver(prob, opts)
# initial_controls!(ilqr, U0)
# solve!(ilqr)

# X = states(ilqr)
# U = controls(ilqr)

# #plot position over time and volecity over time
# plot3 = plot(1:N, [x[1] for x in X], label="p")
# plot!(1:N, [x[2] for x in X], label="theta")
# plot!(1:N, [x[3] for x in X], label="p_dot")
# plot!(1:N, [x[4] for x in X], label="theta_dot")

# # on a separate plot, plot control over time
# plot4 = plot(1:N-1, [u[1] for u in U], label="Control")


# display(plot3)
# display(plot4)
