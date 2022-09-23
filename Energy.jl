#Calculate the Energy E[a.traj]
#E_BioMech(a::agent, p1::NTuple{2, Float64}, p2::NTuple{2, Float64}) = a.α*d(p1, p2)
#E_BioMech(a::agent, p1::NTuple{2, Float64}, p2::NTuple{2, Float64}) =
#    a.α*(δt*(1+a.ϵ))*(d(p1, p2)/(δt*(1+a.ϵ))-a.v_des)^2#
#E_BioMech(a::agent, p1::NTuple{2, Float64}, p2::NTuple{2, Float64}) = a.α*d(p1, p2)^2
E_BioMech(a::agent, P1::NTuple{2, Float64}, P2::NTuple{2, Float64}) = a.α*a.δt*((1+a.ϵ)*f(v(a, P1, P2))+a.ϵ*g(a))

#fitted cost for walking with vel v
f(v) = v < 0.1 ? 7.6*v-35.4*v^2 : 0.4+0.6*v^2
df(v) = v < 0.1 ? 7.6-70.8*v : 1.2*v

#derived function for gain walking at v_des?
g(a::agent) = a.ϵ*(a.v_des*df(a.v_des)-f(a.v_des))

E_CV(a::agent, P1, P2, P3) = a.β*abs(∠(e_(P1, P2), e_(P2, P3)))/(π)

function E(a::agent, obstacle::geometry)

    E_ = E(a,  a.start, a.traj[1], a.traj[2], obstacle)

    for i in 2:length(a.traj)-1

        E_ += E(a,  a.traj[i-1], a.traj[i], a.traj[i+1], obstacle)

    end

    E_ + E(a,  a.traj[end-1], a.traj[end], a.goal, obstacle) + E(a, a.traj[end], a.goal, obstacle)

end

function E(a::agent, P1, P2, P3, obstacle::geometry)

    v(a, P1, P2)*η(a, obstacles, P2)+E_BioMech(a, P1, P2)+E_CV(a, P1, P2, P3)

end

function E(a::agent, P1, P2, obstacle::geometry)

    v(a, P1, P2)*η(a, obstacles, P2)+E_BioMech(a, P1, P2)
    #0.0

end




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

        ∇E_mid(a, a.start, a.traj[1], a.traj[2], a.traj[3], a.goal, obstacle)

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
