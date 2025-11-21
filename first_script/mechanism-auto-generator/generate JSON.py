import json

class Linkage:
    def __init__(self, start,end):
        self.start = start
        self.end = end
    def to_dict(self):
        return{
            "start":self.start,
            "end":self.end
        }


start=[0,0]
end=[3,8]
my_linkage=Linkage(start,end)
my_dict=my_linkage.to_dict()


# Specify the file path
file_path = "linkage_params.json"

with open(file_path, 'w') as json_file:
    json.dump(my_dict, json_file, indent=4) # indent for pretty-printing

print(f"JSON data successfully written to {file_path}")