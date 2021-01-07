function Td = place_lead(w, N)
    % Compute Td to place the maximum phase contribution of the lead
    % compensator at w (N determines the second corner frequency of the
    % compensator.
    Td = N/w/sqrt(N + 1);
end