import warp as wp


class TuplePair():
    """Tuple pair has type (tuple, data: any) where data is readable and writable
    """
    def __init__(self, t: tuple, data:wp.array = None):
        self._t = t
        self.data: wp.array = data

    
    # get item from tuple
    def __getitem__(self, key):
        return self._t[key]
    

if __name__ == "__main__":
    t = ("hello", 'a', 1)
    val = TuplePair(t, None)
    for i in range(len(t)):
        print(val[i])

    val.data = [1, 2, 3]
    print(val.data)
    val.data = None
    print(val.data)