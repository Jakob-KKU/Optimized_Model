#Calculate the Field η
function HelloWorld()
    println("HI")
end
#gaussian and its gradient at point x
G(x::NTuple{2, Float64}, μ::NTuple{2, Float64}, σ::Float64) = 1/(2*π*(σ)^2)*exp(-(d(x, μ)^2)/(2*(σ)^2))
∇G(x::NTuple{2, Float64}, μ::NTuple{2, Float64}, σ::Float64) = -1 .* G(x, μ, σ).*d_vec(x, μ)./(1.5*σ)^2

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


#only used for plotting η
function Calc_η_Matrix(x, y, obstacles)

    η_matrix = fill(0.0, length(x), length(y))

    for i in 1:length(x)

        for j in 1:length(y)

            η_matrix[j, i] = η((x[i], y[j]), obstacles)

        end

    end

    η_matrix

end
