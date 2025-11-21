import json

class Linkage:
    def __init__(self,position_matrix,connection_matrix):
        self.pos=position_matrix
        self.con=connection_matrix
        self.num_links=len(connection_matrix)

    def to_dict(self):
        links={"num_links":self.num_links}
        for i in range(self.num_links):
            links[f'Link{i}']={"start":self.pos[self.con[i][0]],"end": self.pos[self.con[i][1]]}

        return links


node1=[0,0]
node2=[3,8]
node3=[8,8]
node4=[8,0]
position_matrix=[node1,node2,node3,node4]
connection_matrix=[[0,1],[1,2],[2,3],[3,0]]
my_linkage=Linkage(position_matrix,connection_matrix)
my_dict=my_linkage.to_dict()


# Specify the file path
file_path = "linkage_params.json"

with open(file_path, 'w') as json_file:
    json.dump(my_dict, json_file, indent=4) # indent for pretty-printing

print(f"JSON data successfully written to {file_path}")