#### Parallel ###

function Run_Parallel_Optimization!(a::agent, obstacle::geometry, steps::Int, Δsteps::Int, ΔE)

    #show progress
    q = Progress(steps, 0.1)

    trajectories = fill((0.0, 0.0), Int(round(steps/Δsteps)+1), length(a.traj))
    energies = fill(0.0, steps+1)

    energies[1] = E(a, obstacle)
    trajectories[1, :] .= a.traj
    j, k = 1, steps+1

    ϵ_temp, traj_temp = 0.0, fill((0.0, 0.0), length(a.traj))

    for i in 1:steps

        ϵ_temp, traj_temp =  Calc_Parallel!(a, obstacle, ϵ_temp, traj_temp)

        Update_Parallel!(a, ϵ_temp, traj_temp)


        if mod(i, Δsteps) == 0
            trajectories[j, :] .= a.traj
            j+=1
        end

        energies[i+1] = E(a, obstacle)

        if abs(energies[i+1] - energies[i]) < ΔE

            println("Convergence criterum was met at step ", i, "!")
            k = i-1
            trajectories[j, :] .= a.traj

            break
        end

        next!(q)

    end

    trajectories[1:j, :], energies[1:k]

end

function Calc_Parallel!(a::agent, obstacle::geometry, ϵ_temp, traj_temp)

    ϵ_temp = Calc_ϵ(a, obstacle::geometry)

    for site in 1:length(a.traj)

        traj_temp[site] = Calc_at(site, a, obstacle)

    end

    ϵ_temp, traj_temp

end

function Calc_at(site::Int, a::agent, obstacle::geometry)

    a.traj[site] .- a.γ .* ∇E(a, site, obstacle)

end

function Calc_ϵ(a::agent, obstacle::geometry)

    a.ϵ - a.γ*∇E_ϵ(a, obstacle)

end

function Update_Parallel!(a::agent, ϵ_temp, traj_temp)

    a.ϵ = ϵ_temp
    a.traj = traj_temp

end
