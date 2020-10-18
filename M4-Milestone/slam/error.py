import json
f = open('slam.json')
data = json.load(f)
x_coord = data["map"][0]
y_coord = data["map"][1]
map_ids = data["AR_tag_list"]
x_real = [-1.274707,-1.075032, -0.50128,2.074430,3.11284,1.472976]
y_real = [-0.458379,4.12235,1.538357,4.07726,0.882415,1.574148]
distance_dict = {}
for i in range(len(map_ids)):
    if (map_ids[i] == 1):
        distance_dict["1"] = (((x_real[0]-x_coord[i])**2+(y_real[0] - y_coord[i])**2)**0.5)
    elif (map_ids[i] == 3):
        distance_dict["3"] =(((x_real[1]-x_coord[i])**2+(y_real[1] - y_coord[i])**2)**0.5)
    elif (map_ids[i] == 5):
        distance_dict["5"] =(((x_real[2]-x_coord[i])**2+(y_real[2] - y_coord[i])**2)**0.5)
    elif (map_ids[i] == 7):
        distance_dict["7"] =(((x_real[3]-x_coord[i])**2+(y_real[3] - y_coord[i])**2)**0.5)
    elif (map_ids[i] == 9):
        distance_dict["9"] =(((x_real[4]-x_coord[i])**2+(y_real[4] - y_coord[i])**2)**0.5)
    else:
        distance_dict["11"] =(((x_real[5]-x_coord[i])**2+(y_real[5] - y_coord[i])**2)**0.5)

print(distance_dict)
