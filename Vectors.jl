#Vectors
e_(a::NTuple{2, Float64}, b::NTuple{2, Float64}) = (b.-a)./d(a, b)

d(u::NTuple{2, Float64}, v::NTuple{2, Float64}) =  sqrt((u[1]-v[1])^2+(u[2]-v[2])^2)

d_vec(a::NTuple{2, Float64}, b::NTuple{2, Float64}) = (b.-a)

⋅(u::NTuple{2, Float64}, v::NTuple{2, Float64}) = u[1]*v[1]+u[2]*v[2]

normalize(a::NTuple{2, Float64}) = a./abs(a)

Base.abs(a::NTuple{2, Float64}) = sqrt(a[1]^2+a[2]^2)

∇d(u::NTuple{2, Float64}, v::NTuple{2, Float64}) = -1 .* (u.-v) ./ d(u, v)
