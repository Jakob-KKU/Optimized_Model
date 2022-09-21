#Calculate the Field η

#gaussian and its gradient at point x
G(x::NTuple{2, Float64}, μ::NTuple{2, Float64}, σ::Float64) = 1/(2*π*σ^2)*exp(-(d(x, μ)^2)/(2*σ^2))
∇G(x::NTuple{2, Float64}, μ::NTuple{2, Float64}, σ::Float64) = G(x, μ, σ).*d_vec(x, μ)./σ^2

#Collision Probability
P_C(a::agent, o::element, P1::NTuple{2, Float64}) = exp(-d_sq(P1, o.x)/(2*a.σ^2))*(1-exp(-r(a, o)^2/(2*a.σ^2)))
r(a::agent, o::element) = (a.l + o.l)/2

function η(x::NTuple{2, Float64}, obstacle::geometry)

    η_ = 0.0

    for o in obstacle.element

        η_ += G(x, o.x, o.l)

    end

    η_

end

function ∇η(x::NTuple{2, Float64}, obstacle::geometry)

    ∇η_ = (0.0, 0.0)

    for o in obstacle.element

        ∇η_ = ∇η_ .+ ∇G(x, o.x, o.l)

    end

    ∇η_

end

#based on collision Probabilities P_C
function η(a::agent, obstacles::geometry, P1::NTuple{2, Float64})

    η_ = 0.0

    for (i,o) in enumerate(obstacles.element)

        P_C_ = P_C(a, o, P1)
        η_ += P_C_

        if P_C_ > 0.05

            for o_ in obstacles.element[i+1:end]

               η_ += -P_C(a, o_, P1)*P_C(a, o, P1)

            end

        end

    end

    η_

end


#only used for plotting η
function Calc_η_Matrix(a::agent, obstacles::geometry, x, y)

    η_matrix = fill(0.0, length(y), length(x))

    for i in 1:length(x)

        for j in 1:length(y)

            η_matrix[j, i] = η(a, obstacles, (x[i], y[j]))

        end

    end

    η_matrix

end
