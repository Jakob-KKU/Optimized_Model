#Init the Agent

function Create_Agent(start, goal, δt, T, α, β, ϵ, γ)

    steps = Int(floor(T/δt))

    agent(start, start, goal, fill((0.0, 0.0), steps), α, β, ϵ, δt, T, γ)

end

function Init_Parameter!(a::agent, δt, T, α, β, ϵ, γ, start, goal)
    a.δt = δt
    a.T = T
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
