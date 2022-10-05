#create crowd
function Create_Crowd(N::Int, steps)
    crowd([agent((0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0),
    fill((0.0, 0.0), steps[i]),
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) for i in 1:N])
end

Steps_Traj(start::Tuple{Float64, Float64}, goal::Tuple{Float64, Float64}, v_des, δt) = Int(floor(T(start, goal, v_des)/δt))
Steps_Traj(start::Vector{Tuple{Float64, Float64}}, goal::Vector{Tuple{Float64, Float64}}, v_des::Float64, δt::Float64)  =
    [Steps_Traj(start[i], goal[i], v_des, δt) for i in 1:length(start)]

#Init the Agent
function Create_Agent(start, goal, δt, v_des, α, β, σ, ϵ, l, γ)

    steps = Int(floor(T(start, goal, v_des)/δt))

    agent(start, (0.0, 0.0), start, goal, fill((0.0, 0.0), steps), α, β, σ, ϵ, δt, v_des, l, γ)

end

function Create_Agent(start, im_goal, goal, δt, v_des, α, β, σ, ϵ, l, γ)

    steps = Int(floor(T(start, im_goal, goal, v_des)/δt))

    agent(start, (0.0, 0.0), start, goal, fill((0.0, 0.0), steps), α, β, σ, ϵ, δt, v_des, l, γ)

end

function Init_Parameter!(a::agent, δt, v_des, α, β, σ, ϵ, γ, l, start, goal)
    a.δt = δt
    a.v_des = v_des
    a.α = α
    a.β = β
    a.σ = σ
    a.ϵ = ϵ
    a.l = l
    a.goal = goal
    a.start = start
    a.γ = γ
end

function Init_Hom_Parameters!(crowd::crowd, δt, v_des, α, β, σ, ϵ, γ, l, start, goal)

    for (i, a) in enumerate(crowd.agent)

        Init_Parameter!(a, δt, v_des, α, β, σ, ϵ, γ, l, start[i], goal[i])

    end

end


#A line from start to goal
function Init_Initial_Guess!(a::agent)

    d_ = d(a.start, a.goal)/(length(a.traj)+1)

    for (i,x) in enumerate(a.traj)
        a.traj[i] = a.start .+ d_ .* i .* e_(a.start, a.goal)
    end

end

function Init_Initial_Guess!(crowd::crowd)

    for a in crowd.agent

        Init_Initial_Guess!(a)

    end

end

function Init_Initial_Velocity!(crwd::crowd)

    for a in crwd.agent

        a.v = (a.traj[2].-a.traj[1])./δt

    end

end

#line from start to im_goal to goal
function Init_Initial_Guess_Corner!(a::agent, im_goal)

    N2 = Int(round(length(a.traj)/2))
    T2 = T(a.start, im_goal, a.goal, a.v_des)/2

    for (i,x) in enumerate(a.traj[1:N2])
        a.traj[i] = a.start .+ d(a.start, im_goal)/(T2)*(i*a.δt) .* e_(a.start, im_goal)
    end

    for (i,x) in enumerate(a.traj[N2+1:end])
        a.traj[N2+i] = im_goal .+ d(im_goal, a.goal)/(T2)*(i*a.δt) .* e_(im_goal, a.goal)
    end

end
