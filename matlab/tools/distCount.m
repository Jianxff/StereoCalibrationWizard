function dist = distCount(P1,P2)
    dist = 0;
    if length(P1) == 2
        dist = sqrt((P1(1) - P2(1)).^2 + (P1(2) - P2(2)).^2);
    end
    if length(P1) == 3
        dist = sqrt((P1(1) - P2(1)).^2 + (P1(2) - P2(2)).^2 + (P1(3) - P2(3)).^2);
    end
end