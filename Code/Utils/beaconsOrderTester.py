#pink green blue red


angles = [[0, 90, 180, 270],
[270, 0, 90, 180]]


# Check the order of the beacon angles
valid_order = False

def isInBetween(A,B,C, clockwise = False):
    # it's like A <= B and B <= C, but for angles which means there are modulo 
    # B is the angle we want to check is in between A and C

    # if clockwise:
    #     if A <= C: # doesn't work
    #         return (B > A and B < C)
    #     else:
    #         return (B < A or B > C) 
    # else:
    if A <= C:
        return (B > A or B < C)
    else:
        return (B < A and B > C)


print(isInBetween( 210,127,45,  clockwise=False))
print(isInBetween( 210,127,308,  clockwise=False))
print(isInBetween( 210,45,308,  clockwise=False))
print(isInBetween( 127,45,308,  clockwise=False))
print(isInBetween( 277,175,5,  clockwise=False))