class State(object):
    """
    state class : node + selfloop label
    """
    def __init__(self, it, label=''):
        self.x = it[0]
        self.q = it[1]
        self.label = label

    def __eq__(self, other):
        return (self.x == other.x) and (self.q == other.q)

    def __hash__(self):
        # set invovke
        return hash((self.x, self.q))

    def __str__(self):
        return '{0},{1}'.format(self.x, self.q)

    def xq(self):
        return self.x, self.q
