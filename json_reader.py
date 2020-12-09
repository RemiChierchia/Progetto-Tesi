import os, json
import pandas as pd

# this finds our json files
#path_to_json = '/home/Documents/progetto\tesi/laterale_rallenty/'
json_files = [pos_json for pos_json in os.listdir() if pos_json.endswith('.json')]

# here I define my pandas Dataframe with the columns I want to get from the json
jsons_data = pd.DataFrame(columns=['pose', 'left_hand', 'right_hand'])
jsons_ouput = pd.DataFrame(columns=['pose', 'left_hand', 'right_hand'])
# we need both the json and an index number so use enumerate()

for i,filename in enumerate(sorted(json_files)):

    with open(os.path.join(filename)) as json_file:
        json_text = json.load(json_file)

        #if filename >= 'laterale_rallenty_000000000480_keypoints' and filename <= 'laterale_rallenty_000000000888_keypoints':
        if i >= 480 and i <= 888:

            pose = json_text['people'][0]['pose_keypoints_2d']
            left_hand = json_text['people'][0]['hand_left_keypoints_2d']
            right_hand = json_text['people'][0]['hand_right_keypoints_2d']

            jsons_data.loc[i-480] = [pose, left_hand, right_hand] #the index is 'laterale_rallenty_00000000xxxx_keypoints'


jsons_data.to_json('player_move.json')
