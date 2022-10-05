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

function E(a::agent, crwd::crowd, obstacle::geometry)

    E_ = E(a,  a.start, a.traj[1], a.traj[2],  a.δt, crwd, obstacle)

    for i in 2:length(a.traj)-1

        E_ += E(a,  a.traj[i-1], a.traj[i], a.traj[i+1], i*a.δt, crwd, obstacle)

    end

    E_ + E(a,  a.traj[end-1], a.traj[end], a.goal, length(a.traj)*a.δt, crwd, obstacle) +
     E(a, a.traj[end], a.goal, length(a.traj)*a.δt, crwd, obstacle)

end

function E(a::agent, P1, P2, P3, obstacle::geometry)

    v(a, P1, P2)*η(a, obstacles, P2)+E_BioMech(a, P1, P2)+E_CV(a, P1, P2, P3)

end

function E(a::agent, P1, P2, P3, t, crwd::crowd, obstacle::geometry)

    v(a, P1, P2)*η(a, crwd, obstacles, P2, t)+E_BioMech(a, P1, P2)+E_CV(a, P1, P2, P3)

end

function E(a::agent, P1, P2, obstacle::geometry)

    v(a, P1, P2)*η(a, obstacles, P2)+E_BioMech(a, P1, P2)

end

function E(a::agent, P1, P2, t, crwd::crowd, obstacle::geometry)

    v(a, P1, P2)*η(a, crwd, obstacles, P2, t)+E_BioMech(a, P1, P2)

end
