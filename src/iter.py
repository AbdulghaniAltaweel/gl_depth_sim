# -*- coding: utf-8 -*-
#%%
import os

myExtension = ".obj"
myPath = "/home/ai/Downloads/testDataGen"
#%%
#create an new List to include the Names of all object-mesh files
listNames = []
listNames = os.listdir(myPath)

#%%
#function to extract the extension and the file names
from os.path import splitext
def splitext_(path):
    if len(path.split('.')) > 2:
        return path.split('.')[0],'.'.join(path.split('.')[-2:])
    return splitext(path)

#%%
for file_ in listNames:
    file_name,extension = splitext_(file_)
    if extension == myExtension:
        print (file_name)

print( '\n',  len(listNames))

#%%
