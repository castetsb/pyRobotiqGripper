def sign(value):
    """Return the sign of a value
    
    Args:
        - value : Value for which the sign have to be evaluated.
    
    Returns:
        - valueSign : Sign of the value. 1 if positif. -1 if negative.
    """
    valueSign = (value > 0) - (value < 0)

    return valueSign

def listIdValueUnderThreshold(lst,threshold):
    """Return the list id of the first value under a given threshold.

    Args:
        - lst (list): list of values
        - threshold: Threshold under which should be the serached value
    
    Returns:
        - lstIf (int): List id of the first value under a given threshold
    """
    i=0
    lstId=0
    found=False
    while i<len(lst) and not found:
        if lst[i]<threshold:
            lstId=i
            found=True
        i+=1
    if not found:
        raise Exception("Could not find value under {} in {}".format(threshold,lst))
    return lstId

def listSubstract(lst,value):
    """Substract a value to all values of a list.

    Args:
        - lst (list): list of values
        - value: value to substract to all values of the list
    """
    for i in range(len(lst)):
        lst[i] -= value

def areValueIdentical(lst):
    """Return True if all values of the list are identical, False otherwise.

    Args:
        - lst (list): list of values
    
    Returns:
        - res (bool): True if all values of the list are identical, False otherwise.
    """
    value = lst[0]
    res=True
    i=0
    while i<len(lst):
        if lst[i]!=value:
            res=False
        i+=1
    return res

def updateList(lst,value):
    """Shift all values of the given list of 1 to the right and set the given value as the
    first value of the list

    Args:
        - lst (list): list to be updated
        - value: value to set a the beginning of the list
    """
    lst[1:]=lst[:-1]
    lst[0]=value