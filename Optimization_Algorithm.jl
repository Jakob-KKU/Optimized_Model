#Optimization Algorithm: site by site from start to goal
function Run_Ordered_Optimization(a::agent, obstacle::geometry, steps::Int, Δsteps::Int)

    trajectories = fill((0.0, 0.0), Int(round(steps/Δsteps)+1), length(a.traj))
    energies = fill(0.0, steps+1)

    energies[1] = E(a, obstacle)
    trajectories[1, :] .= a.traj
    j = 2

    for i in 1:steps

        Update_Ordered!(a, obstacle)

        if mod(i, Δsteps) == 0
            trajectories[j, :] .= a.traj
            j+=1
        end

        energies[i+1] = E(a, obstacle)



    end

    trajectories, energies

end

function Run_Ordered_Optimization(a::agent, obstacle::geometry, steps::Int, Δsteps::Int, ΔE)

    trajectories = fill((0.0, 0.0), Int(round(steps/Δsteps)+1), length(a.traj))
    energies = fill(0.0, steps+1)

    energies[1] = E(a, obstacle)
    trajectories[1, :] .= a.traj
    j, k = 1, steps+1


    for i in 1:steps

        Update_Ordered!(a, obstacle)

        if mod(i, Δsteps) == 0
            j+=1
            trajectories[j, :] .= a.traj

        end


        energies[i+1] = E(a, obstacle)

        if abs(energies[i+1] - energies[i]) < ΔE

            println("Convergence criterum was met at step ", i, "!")
            k = i-1
            trajectories[j, :] .= a.traj

            break
        end

    end

    trajectories[1:j, :], energies[1:k]

end

function Update_Ordered!(a::agent, obstacle::geometry)

    Update_ϵ!(a, obstacle::geometry)

    for site in 1:length(a.traj)

        Update_at!(site, a, obstacle)

    end

end

function Update_at!(site::Int, a::agent, obstacle::geometry)

    a.traj[site] = a.traj[site] .- a.γ .* ∇E(a, site, obstacle)

end

function Update_ϵ!(a::agent, obstacle::geometry)

    a.ϵ += -a.γ*∇E_ϵ(a, obstacle)

end

# in each step T/δt random sites are updated ~ random sequential update
function Run_Random_Optimization(a::agent, obstacle::geometry, steps::Int, Δsteps::Int)

    trajectories = fill((0.0, 0.0), Int(round(steps/Δsteps)+1), length(a.traj))
    trajectories[1, :] .= a.traj
    j = 2

    for i in 1:steps

        Update_Random!(a, obstacle)

        if mod(i, Δsteps) == 0

            trajectories[j, :] .= a.traj
            j+=1
        end

    end

    trajectories

end

function Update_Random!(a::agent, obstacle::geometry)

    for site in 1:length(a.traj)

        Update_at_random!(a, obstacle)

    end

end

function Update_at_random!(a::agent, obstacle::geometry)

    site = rand(1:length(a.traj))

    a.traj[site] = a.traj[site] .+ a.γ .* ∇E(a, site, obstacle)

end
;
