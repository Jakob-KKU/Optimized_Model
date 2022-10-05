
#Calculate the Energy gradient at a point of a.traj
function ∇E(a::agent, i::Int, obstacle::geometry)

    if i == 1

        ∇E(a, a.start, a.traj[1], a.traj[2], obstacle)

    elseif i == length(a.traj)

        ∇E(a, a.traj[end-1], a.traj[end], a.goal, obstacle)

    else

        ∇E(a, a.traj[i-1], a.traj[i], a.traj[i+1], obstacle)

    end

end

#numerical gradient where we assume that only two terms of E, i.e. i & i+1 depend on x_i
#Calculate the Energy gradient at a point of a.traj
function ∇E(a::agent, i::Int, obstacle::geometry)

    if i == 1

        ∇E_start(a, obstacle)

    elseif i == 2

        ∇E_mid(a, a.start, a.traj[1], a.traj[2], a.traj[3], a.traj[4], obstacle)

    elseif i == length(a.traj)

        ∇E_end(a, obstacle)

    elseif i == length(a.traj)-1

        ∇E_mid(a, a.traj[end-3], a.traj[end-2], a.traj[end-1], a.traj[end], a.goal, obstacle)

    else

        ∇E_mid(a, a.traj[i-2], a.traj[i-1], a.traj[i], a.traj[i+1], a.traj[i+2], obstacle)

    end

end

function ∇E_mid(a::agent, P1, P2, P3, P4, P5, obstacle::geometry)

    h = 0.00001

    E_P3 = E(a, P1, P2, P3, obstacle) + E(a, P2, P3, P4, obstacle) + E(a, P3, P4, P5, obstacle)

    P3_x = P3 .+ h.*(1.0, 0.0)
    E_P3_x = E(a, P1, P2, P3_x, obstacle) + E(a, P2, P3_x, P4, obstacle) + E(a, P3_x, P4, P5, obstacle)

    P3_y = P3 .+ h.*(0.0, 1.0)
    E_P3_y = E(a, P1, P2, P3_y, obstacle) + E(a, P2, P3_y, P4, obstacle) + E(a, P3_y, P4, P5, obstacle)

    ((E_P3_x, E_P3_y) .- E_P3) ./ h

end

function ∇E_start(a::agent, obstacle::geometry)

    P1, P2, P3, P4 = a.start, a.traj[1], a.traj[2], a.traj[3]

    h = 0.00001

    E_P2 = E(a, P1, P2, P3, obstacle) + E(a, P2, P3, P4, obstacle)

    P2_x = P2 .+ h.*(1.0, 0.0)
    E_P2_x = E(a, P1, P2_x, P3, obstacle) + E(a, P2_x, P3, P4, obstacle)

    P2_y = P2 .+ h.*(0.0, 1.0)
    E_P2_y = E(a, P1, P2_y, P3, obstacle) + E(a, P2_y, P3, P4, obstacle)

    ((E_P2_x, E_P2_y) .- E_P2) ./ h

end

function ∇E_end(a::agent, obstacle::geometry)

    P1, P2, P3, P4 = a.traj[end-2], a.traj[end-1], a.traj[end], a.goal

    h = 0.00001

    E_P3 = E(a, P1, P2, P3, obstacle) + E(a, P2, P3, P4, obstacle)  + E(a, P3, P4, obstacle)

    P3_x = P3 .+ h.*(1.0, 0.0)
    E_P3_x = E(a, P1, P2, P3_x, obstacle) + E(a, P2, P3_x, P4, obstacle) + E(a, P3_x, P4, obstacle)


    P3_y = P3 .+ h.*(0.0, 1.0)
    E_P3_y = E(a, P1, P2, P3_y, obstacle) + E(a, P2, P3_y, P4, obstacle) + E(a, P3_y, P4, obstacle)

    ((E_P3_x, E_P3_y) .- E_P3) ./ h

end

function ∇E_alternativ(a::agent, i::Int, obstacle::geometry)

    h = 0.00001

    a.traj[i] = a.traj[i] .+ h.*(1.0, 0.0)
    E_x = E(a, obstacle)

    a.traj[i] = a.traj[i] .- h.*(1.0, 0.0) .+ h.*(0.0, 1.0)
    E_y = E(a, obstacle)

    a.traj[i] = a.traj[i] .- h.*(0.0, 1.0)

    ((E_x, E_y) .- E(a, obstacle)) ./ h

end




function ∇E_ϵ(a::agent, obstacle::geometry)


    ∇E_ϵ_ = ∇E_ϵ(a, a.start, a.traj[1], obstacle)

    for i in 2:length(a.traj)

        ∇E_ϵ_ += ∇E_ϵ(a, a.traj[i-1], a.traj[i], obstacle)

    end

    ∇E_ϵ_

end

function ∇E_ϵ(a::agent, P1, P2, obstacle::geometry)

    h = 0.00001

    E_0 = E(a, P1, P2, obstacle)

    a.ϵ += h

    E_1 = E(a, P1, P2, obstacle)

    a.ϵ += -h

    (E_1-E_0)/h

end

#numerical gradient where we assume that only two terms of E, i.e. i & i+1 depend on x_i
#Calculate the Energy gradient at a point of a.traj
function ∇E(a::agent, i::Int, crwd::crowd, obstacle::geometry)

    if i == 1

        ∇E_start(a, t(a, i), crwd, obstacle)

    elseif i == 2

        ∇E_mid(a, a.start, a.traj[1], a.traj[2], a.traj[3], a.traj[4], t(a, i), crwd, obstacle)

    elseif i == length(a.traj)

        ∇E_end(a, t(a, i), crwd, obstacle)

    elseif i == length(a.traj)-1

        ∇E_mid(a, a.traj[end-3], a.traj[end-2], a.traj[end-1], a.traj[end], a.goal, t(a, i), crwd, obstacle)

    else

        ∇E_mid(a, a.traj[i-2], a.traj[i-1], a.traj[i], a.traj[i+1], a.traj[i+2], t(a, i), crwd, obstacle)

    end

end

function ∇E_mid(a::agent, P1, P2, P3, P4, P5, t, crwd::crowd, obstacle::geometry)

    h = 0.00001

    E_P3 = E(a, P1, P2, P3, t, crwd, obstacle) + E(a, P2, P3, P4, t, crwd, obstacle) + E(a, P3, P4, P5, t, crwd, obstacle)

    P3_x = P3 .+ h.*(1.0, 0.0)
    E_P3_x = E(a, P1, P2, P3_x, t, crwd, obstacle) + E(a, P2, P3_x, P4, t, crwd, obstacle) + E(a, P3_x, P4, P5, t, crwd, obstacle)

    P3_y = P3 .+ h.*(0.0, 1.0)
    E_P3_y = E(a, P1, P2, P3_y, t, crwd, obstacle) + E(a, P2, P3_y, P4, t, crwd, obstacle) + E(a, P3_y, P4, P5, t, crwd, obstacle)

    ((E_P3_x, E_P3_y) .- E_P3) ./ h

end

function ∇E_start(a::agent, t, crwd::crowd, obstacle::geometry)

    P1, P2, P3, P4 = a.start, a.traj[1], a.traj[2], a.traj[3]

    h = 0.00001

    E_P2 = E(a, P1, P2, P3, t, crwd, obstacle) + E(a, P2, P3, P4, t, crwd, obstacle)

    P2_x = P2 .+ h.*(1.0, 0.0)
    E_P2_x = E(a, P1, P2_x, P3, t, crwd, obstacle) + E(a, P2_x, P3, P4, t, crwd, obstacle)

    P2_y = P2 .+ h.*(0.0, 1.0)
    E_P2_y = E(a, P1, P2_y, P3, t, crwd, obstacle) + E(a, P2_y, P3, P4, t, crwd, obstacle)

    ((E_P2_x, E_P2_y) .- E_P2) ./ h

end

function ∇E_end(a::agent, t, crwd::crowd, obstacle::geometry)

    P1, P2, P3, P4 = a.traj[end-2], a.traj[end-1], a.traj[end], a.goal

    h = 0.00001

    E_P3 = E(a, P1, P2, P3, t, crwd, obstacle) + E(a, P2, P3, P4, t, crwd, obstacle)  + E(a, P3, P4, t, crwd, obstacle)

    P3_x = P3 .+ h.*(1.0, 0.0)
    E_P3_x = E(a, P1, P2, P3_x, t, crwd, obstacle) + E(a, P2, P3_x, P4, t, crwd, obstacle) + E(a, P3_x, P4, t, crwd, obstacle)


    P3_y = P3 .+ h.*(0.0, 1.0)
    E_P3_y = E(a, P1, P2, P3_y, t, crwd, obstacle) + E(a, P2, P3_y, P4, t, crwd, obstacle) + E(a, P3_y, P4, t, crwd, obstacle)

    ((E_P3_x, E_P3_y) .- E_P3) ./ h

end


function ∇E_ϵ(a::agent, crwd::crowd, obstacle::geometry)


    ∇E_ϵ_ = ∇E_ϵ(a, a.start, a.traj[1], a.δt, crwd, obstacle)

    for i in 2:length(a.traj)

        ∇E_ϵ_ += ∇E_ϵ(a, a.traj[i-1], a.traj[i], i*a.δt, crwd, obstacle)

    end

    ∇E_ϵ_

end

function ∇E_ϵ(a::agent, P1, P2, t, crwd::crowd, obstacle::geometry)

    h = 0.00001

    E_0 = E(a, P1, P2, t, crwd, obstacle)

    a.ϵ += h

    E_1 = E(a, P1, P2, t, crwd, obstacle)

    a.ϵ += -h

    (E_1-E_0)/h

end
