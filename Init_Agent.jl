#Init the Agent

function Create_Agent(start, goal, δt, v_des, α, β, ϵ, γ)

    steps = Int(floor(T(start, goal, v_des)/δt))

    agent(start, start, goal, fill((0.0, 0.0), steps), α, β, ϵ, δt, v_des, γ)

end

function Init_Parameter!(a::agent, δt, v_des, α, β, ϵ, γ, start, goal)
    a.δt = δt
    a.v_des = v_des
    a.α = α
    a.β = β
    a.ϵ = ϵ
    a.goal = goal
    a.start = start
    a.γ = γ
end


#A line from start to goal
function Init_Initial_Guess!(a::agent)

    for (i,x) in enumerate(a.traj)
        a.traj[i] = a.start .+ ((i)*a.δt) .* e_(a.start, a.goal)
    end

end
