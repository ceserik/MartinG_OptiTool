n = 146;
data = repmat([1 0 0; -1 0 0], n, 1);  % (2n × 3)
data = reshape(data', 3, 2, []);       % (3 × 2 × n)
data = permute(data, [2, 1, 3])