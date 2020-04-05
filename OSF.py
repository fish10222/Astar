
"""
    Lookup table for OSF
    {
        ∆h: (∆f,f_next)
    }
"""

lookup_table = {
    -1:(0,1),
    0:(1,2),
    1:(1,-1),
}
def OSF (current_loc, h_values):
    answers = ([], 0)
    return answers