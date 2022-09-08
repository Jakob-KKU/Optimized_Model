#Init the struct

mutable struct agent

    x::NTuple{2, Float64}

    start::NTuple{2, Float64}
    goal::NTuple{2, Float64}
    traj::Vector{NTuple{2, Float64}}

    α::Float64 #influence of velocity
    ϵ::Float64 #Dilation parameter of maximum time
    β::Float64 #influence of difference to V_des
    δt::Float64 #time-step
    T::Float64 #maximum time


    γ::Float64 #Optimization stepsize

end


struct element
    x::NTuple{2, Float64}
    l::Float64
end

struct geometry
    element::Array{element, 1}
end
