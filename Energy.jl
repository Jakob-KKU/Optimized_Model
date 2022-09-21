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




function E(a::agent, obstacle::geometry)

    E_ = E(a,  a.start, a.traj[1], obstacle)

    for i in 2:length(a.traj)

        E_ += E(a,  a.traj[i], a.traj[i-1], obstacle)

    end

    E_

end

function E(a::agent,  P1, P2, obstacle::geometry)

    η(a, obstacles, P2) + E_BioMech(a, P1, P2)

end

function E(a::agent, i::Int, obstacles::geometry)

   if i == 1

        η(a, obstacles, a.traj[1]) + E_BioMech(a, a.traj[1], a.start)

    else

        η(a, obstacles, a.traj[i]) + E_BioMech(a, a.traj[i], a.traj[i-1])

    end

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
function ∇E(a::agent, P1, P2, P3, obstacle::geometry)

    h = 0.00001

    E_P2 = E(a, P1, P2, obstacle) + E(a,  P2, P3, obstacle)

    E_P2x = E(a, P1, P2 .+ h.*(1.0, 0.0), obstacle) + E(a, P2 .+ h.*(1.0, 0.0), P3, obstacle)
    E_P2y = E(a, P1, P2 .+ h.*(0.0, 1.0), obstacle) + E(a, P2 .+ h.*(0.0, 1.0), P3, obstacle)

    ((E_P2x, E_P2y) .- E_P2) ./ h

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
