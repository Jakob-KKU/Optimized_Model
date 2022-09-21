#Init the Obstacles

function Create_Geometry(x_Elements::Vector{NTuple{2, Float64}}, l_Elements::Vector{Float64})

    geometry([element(x_Elements[i], l_Elements[i]) for i in 1:length(x_Elements)])

end

function Create_Corridor_with_Obstacle(width, laenge, l, Δx, l_obs)
    geometry_y = vcat(zeros(length(LinRange(0:Δx:laenge))), fill(width, length(LinRange(0:Δx:laenge))), width/2)
    geometry_x = vcat(LinRange(0:Δx:laenge), LinRange(0:Δx:laenge), laenge/2)
    ls = fill(l, length(geometry_x))
    ls[end] = l_obs
    geometry([element((geometry_x[i], geometry_y[i]), ls[i]) for i in 1:length(geometry_x)])
end

function Create_Corner(laenge, l, Δx)
    geometry_y = vcat(fill(laenge, length(LinRange(0:Δx:laenge))), LinRange(laenge:-Δx:0))
    geometry_x = vcat(LinRange(0:Δx:laenge), fill(laenge, length(LinRange(laenge:-Δx:0))))

    geometry([element((geometry_x[i], geometry_y[i]), l) for i in 1:length(geometry_x)])
end
