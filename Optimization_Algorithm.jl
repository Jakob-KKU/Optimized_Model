#Optimization Algorithm
function Run_Ordered_Optimization(a::agent, obstacle::geometry, steps::Int, Δsteps::Int)

    trajectories = fill((0.0, 0.0), Int(round(steps/Δsteps)+1), length(a.traj))
    trajectories[1, :] .= a.traj
    j = 2

    for i in 1:steps

        Update_Ordered!(a, obstacle)

        if mod(i, Δsteps) == 0

            trajectories[j, :] .= a.traj
            j+=1
        end

    end

    trajectories

end

function Update_Ordered!(a::agent, obstacle::geometry)

    for site in 1:length(a.traj)

        Update_at!(site, a, obstacle)

    end

end

function Update_at!(site::Int, a::agent, obstacle::geometry)

    a.traj[site] = a.traj[site] .+ a.γ .* ∇E(a, site, obstacle)

end


function Update_at_random!(a::agent, obstacle::geometry)

    site = rand(1:length(a.traj))

    a.traj[site] = a.traj[site] .+ a.γ .* ∇E(a, site, obstacle)

end
;
