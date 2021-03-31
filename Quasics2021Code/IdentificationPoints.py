def identifyPathForChallenge(left_list, width_list):

    numBalls = len(width_list)

    # Find the largest ball: assumption is that it should be right in front of us
    index = -1 
    bestIndex = -1 
    for width in width_list:
        index = index + 1

        # Determines if the current width beats the "best" width curretly. 

        if bestIndex == -1 or width_list[index] > width_list[bestIndex]: 
            bestIndex = index

    print("Best Index is {}".format(bestIndex))

    # Figure out which path is set up, based on positions of the balls
    # relative to the largest one.  (And how many we can "see".)
    if numBalls == 3:
        # Must be Path A.  Just need to figure out which route within it.
        if bestIndex == 0:
            otherindx1 = 1
            otherindx2 = 2

        elif bestIndex == 1:
            otherindx1 = 0
            otherindx2 = 2

        elif bestIndex == 2:
            otherindx1 = 0 
            otherindx2 = 1

        if left_list[otherindx1] < left_list[bestIndex] and left_list[otherindx2] < left_list[bestIndex]:
            print("It is Path 2A")

        elif left_list[otherindx1] > left_list[bestIndex] and left_list[otherindx2] > left_list[bestIndex]:
            print("Error")

        else:
            print("It is Path 1A")

    elif numBalls == 2:
        # Must be path B
        if bestIndex == 0:
            otherindx = 1
        else:
            otherindx = 0

        if left_list[otherindx] > left_list[bestIndex]:
            print("It is Path 1B")
        else:
            print("It is Path 2B")

    else:
        # Something weird, can't determine A or B
        print("Can't identify primary path information!")



# all_targets_left_list = [17, 105, 75]
all_targets_left_list = [17, 105, 75]
all_targets_top_list = [35, 35, 37]
all_targets_width_list = [7, 8, 14]
all_targets_height_list = [6, 8, 14]

identifyPathForChallenge(all_targets_left_list, all_targets_width_list)
