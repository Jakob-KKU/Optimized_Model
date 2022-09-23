#Init the struct

mutable struct agent

    x::NTuple{2, Float64}

    start::NTuple{2, Float64}
    goal::NTuple{2, Float64}
    traj::Vector{NTuple{2, Float64}}

    α::Float64 #influence of velocity
    β::Float64 #Penalty for curvature
    σ::Float64 #inverse precision of perception
    ϵ::Float64 #Dilation parameter of maximum time
    δt::Float64 #time-step
    v_des::Float64 #desired_velocity
    l::Float64

    γ::Float64 #Optimization stepsize

end


mutable struct element
    x::NTuple{2, Float64}
    l::Float64
end

struct geometry
    element::Array{element, 1}
end
