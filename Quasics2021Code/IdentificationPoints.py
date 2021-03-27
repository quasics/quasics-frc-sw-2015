# all_targets_left_list = [17, 105, 75]
all_targets_left_list = [17, 105, 75]
all_targets_top_list = [35, 35, 37]
all_targets_width_list = [7, 8, 14]
all_targets_height_list = [6, 8, 14]

numBalls = len(all_targets_width_list)


index = -1 
bestIndex = -1 
for width in all_targets_width_list:
    index = index + 1

    # Determines if the current width beats the "best" width curretly. 

    if bestIndex == -1 or all_targets_width_list[index] > all_targets_width_list[bestIndex]: 
        bestIndex = index

print("Best Index is {}".format(bestIndex))

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

    if all_targets_left_list[otherindx1] < all_targets_left_list[bestIndex] and all_targets_left_list[otherindx2] < all_targets_left_list[bestIndex]:
        print("It is Path 2A")

    elif all_targets_left_list[otherindx1] > all_targets_left_list[bestIndex] and all_targets_left_list[otherindx2] > all_targets_left_list[bestIndex]:
        print("Error")

    else:
        print("It is Path 1A")

elif numBalls == 2:
    # Must be path B
    if bestIndex == 0:
        otherindx = 1
    else:
        otherindx = 0

    if all_targets_left_list[otherindx] > all_targets_left_list[bestIndex]:
        print("It is Path 1B")
    else:
        print("It is Path 2B")
    
    

else:
    # Something weird, can't determine A or B
    print("Can't identify primary path information!")
