#Init the Obstacles

function Create_Geometry(x_Elements::Vector{NTuple{2, Float64}}, l_Elements::Vector{Float64})

    geometry([element(x_Elements[i], l_Elements[i]) for i in 1:length(x_Elements)])

end
