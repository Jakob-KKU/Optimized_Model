#Init the Agent

function Create_Agent(start, goal, δt, v_des, α, β, σ, ϵ, l, γ)

    steps = Int(floor(T(start, goal, v_des)/δt))

    agent(start, start, goal, fill((0.0, 0.0), steps), α, β, σ, ϵ, δt, v_des, l, γ)

end

function Create_Agent(start, im_goal, goal, δt, v_des, α, β, σ, ϵ, l, γ)

    steps = Int(floor(T(start, im_goal, goal, v_des)/δt))

    agent(start, start, goal, fill((0.0, 0.0), steps), α, β, σ, ϵ, δt, v_des, l, γ)

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


#A line from start to goal
function Init_Initial_Guess!(a::agent)

    for (i,x) in enumerate(a.traj)
        a.traj[i] = a.start .+ d(a.start, a.goal)/T(a)*(i*a.δt) .* e_(a.start, a.goal)
    end

end

#NOT COMPLETELY Correct
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
