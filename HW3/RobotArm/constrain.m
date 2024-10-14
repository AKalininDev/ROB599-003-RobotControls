function constrainedVal = constrain(value, upperBound, lowerBound)

    if value > upperBound
        constrainedVal = upperBound;
        return;
    end;

    if value < lowerBound
        constrainedVal = lowerBound;
        return;
    end;

    constrainedVal = value;
end

