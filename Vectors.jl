#Vectors
e_(a::NTuple{2, Float64}, b::NTuple{2, Float64}) = normalize(b.-a)

d(u::NTuple{2, Float64}, v::NTuple{2, Float64}) =  sqrt((u[1]-v[1])^2+(u[2]-v[2])^2)

d_vec(a::NTuple{2, Float64}, b::NTuple{2, Float64}) = (b.-a)

d_sq(P1, P2) = (P1[1]-P2[1])^2+(P1[2]-P2[2])^2

⋅(u::NTuple{2, Float64}, v::NTuple{2, Float64}) = u[1]*v[1]+u[2]*v[2]

normalize(a::NTuple{2, Float64}) = a./abs(a)

Base.abs(a::NTuple{2, Float64}) = sqrt(a[1]^2+a[2]^2)

∇d(u::NTuple{2, Float64}, v::NTuple{2, Float64}) = -1 .* (u.-v) ./ d(u, v)

v(P1::NTuple{2, Float64}, P2::NTuple{2, Float64}, P3::NTuple{2, Float64}, δt) = (d(P1, P2) + d(P2, P3))/(2*δt)
v(a::agent, P1, P2) = d(P1, P2)/(a.δt*(1+a.ϵ))


function v(traj::Vector, start, goal, δt)

    vels = fill(0.0, length(traj))

    vels[1] = v(start, traj[1], traj[2], δt)

    for i in 2:length(traj)-1

        vels[i] = v(traj[i-1], traj[i], traj[i+1], δt)

    end

    vels[end] =  v(traj[end-1], traj[end], goal, δt)

    vels

end

#estimated time from start to goal
T(a::agent) = d(a.start, a.goal)/(a.v_des)
T(start, goal, v_des) = d(start, goal)/(v_des)
T(start, im_goal, goal, v_des) = (d(start, im_goal)+d(im_goal, goal))/(v_des)


function ∠(a::NTuple{2, Float64}, b::NTuple{2, Float64})

    x = round((a⋅b)/(abs(a)*abs(b)), digits = 4)

    #if x > 1.0
    #    println("a = ", a, " und b = ", b)
    #    acos(1)
    #elseif x < 1
    #    acos(-1.0)
    #else
        acos(x)
    #end
end
