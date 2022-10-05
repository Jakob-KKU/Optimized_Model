#Calculate the Field η

#Collision Probability
P_C(a::agent, o::element, P1::NTuple{2, Float64}) = exp(-d_sq(P1, o.x)/(2*a.σ^2))*(1-exp(-r(a, o)^2/(2*a.σ^2)))

function P_C(a::agent, b::agent, P1::NTuple{2, Float64}, t)

    if a != b

        exp(-d_sq(P1, x_ant(b, t))/(2*a.σ^2))*(1-exp(-r(a, b)^2/(2*a.σ^2)))

    else

        0.0

    end

end


#based on collision Probabilities P_C
function η(a::agent, crwd::crowd, obstacles::geometry, P1::NTuple{2, Float64}, t::Float64)

    η(a, obstacles, P1) + η(a, crwd, P1, t)

end

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

function η(a::agent, crwd::crowd, P1::NTuple{2, Float64}, t)

    η_ = 0.0

    for (i,b) in enumerate(crwd.agent)

        P_C_ = P_C(a, b, P1, t)
        η_ += P_C_

        if P_C_ > 0.05

            for c in crwd.agent[i+1:end]

                η_ += -P_C(a, c, P1, t)*P_C(a, b, P1, t)

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

function Calc_η_Matrix(a::agent, crwd::crowd, obstacles::geometry, x, y, t)

    η_matrix = fill(0.0, length(y), length(x))

    for i in 1:length(x)

        for j in 1:length(y)

            η_matrix[j, i] = η(a, crwd, obstacles, (x[i], y[j]), t)

        end

    end

    η_matrix

end
