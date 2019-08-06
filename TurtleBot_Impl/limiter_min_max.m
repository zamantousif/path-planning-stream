function [limited_out] = limiter_min_max(input, MIN, MAX)
% LIMITER_MIN_MAX 
% This function limits the input to the MIN and MAX limits
% if MIN <= input <= MAX, limited_out = input
% if input < MIN, limited_out = MIN
% if input > MAX, limited_out = MAX

if input <= MAX
    if input >= MIN
        limited_out = input;
    elseif input < MIN
        limited_out = MIN;
    end
elseif input > MAX
    limited_out = MAX;
end

end