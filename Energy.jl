#Calculate the Energy E[a.traj]
function E(a::agent, obstacle::geometry)

    E_ = η(a.traj[1], obstacle) + a.α*d(a.traj[1], a.start)

    for i in 2:length(a.traj)

        E_ += η(a.traj[i], obstacle) + a.α*d(a.traj[i], a.traj[i-1])

    end

    E_

end

#Calculate the Energy gradient at a point of a.traj
function ∇E(a::agent, i::Int, obstacle::geometry)

    if i == 1

        ∇η(a.traj[i], obstacle) .+ a.α .* (∇d(a.traj[i], start) .- ∇d(a.traj[i+1], a.traj[i]))

    elseif i == length(a.traj)

        ∇η(a.traj[i], obstacle) .+ a.α .* (∇d(a.traj[i], a.traj[i-1]) .- ∇d(a.goal, a.traj[i]))

    else

        ∇η(a.traj[i], obstacle) .+ a.α .* (∇d(a.traj[i], a.traj[i-1]) .- ∇d(a.traj[i+1], a.traj[i]))
    end

end
