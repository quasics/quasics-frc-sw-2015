all_targets_left_list = [75, 105, 17]
all_targets_top_list = [35, 37, 35]
all_targets_width_list = [14, 8, 7]
all_targets_height_list = [14, 8, 6]

numBalls = len(widths)

index = -1 
bestIndex = -1 
best = None
for width in width:
    index = index + 1
    
    # Ignoring small things that can cause noise in the picture, and only using width in the database
    if cv2.width_list(width) < 7
        continue

    # Determines if the current width beats the "best" width curretly. 

    if bestIndex = -1 or computeWidth(contour) > computeWidth(best):
        best = contour 
        bestIndex = index
