#from src.odom_tf import encoder_difference, max_ticks
def encoder_difference(curr_ticks, prev_ticks):
    """
    computes the change in ticks since the last time ticks were counted, accounting for wraparound
    """
    delta_ticks = curr_ticks - prev_ticks
    if abs(delta_ticks) > max_ticks:  # odds are the encoder ticks wrapped around with a difference this large.
        if curr_ticks > prev_ticks:  # wrap from low to high -> negative tick change
            ext_ticks = -max_ticks - (max_ticks - curr_ticks)  # ext_ticks = "extended ticks", a tick count that exceeds max or -max ticks
            print('low to high', ext_ticks)
        else:                        # wrap from high to low -> positive tick change
            ext_ticks = max_ticks + (curr_ticks + max_ticks) 
            print('high to low', ext_ticks)
        delta_ticks = ext_ticks - prev_ticks
    else:
        print('no wrap')
    return delta_ticks

max_ticks = 4294967296  # value set in tivac.h

print(encoder_difference(max_ticks - 5, -max_ticks + 8))  # should print -13
print(encoder_difference(-max_ticks + 3, max_ticks - 4))  # should print 7
print(encoder_difference(max_ticks - 30, max_ticks - 14)) # should print -16