# -*- coding: utf-8 -*-
#%%
import os
from os.path import splitext

#myExtension1 = ".obj"
#myExtension2 = ".dae"
#myExtension3 = ".stl"
myExtensionList = [".obj",".dae", ".stl"]
myPath = "/home/ai/Downloads/testDataGen"

#create an new List to include the Names of all object-mesh files
fullListNames = []
listNames = []
fullListNames = os.listdir(myPath)

#%%
#function to extract the extension and the file names. When Required
def splitext_(path):
    if len(path.split('.')) > 2:
        return path.split('.')[0],'.'.join(path.split('.')[-2:])
    return splitext(path)

#%%
for file_ in fullListNames:
    file_name,extension = splitext_(file_)
#    if extension == myExtension1 or extension == myExtension2 or extension == myExtension3:
    if extension in myExtensionList :
        listNames.append(file_)
       
#%%
print( '\n',  len(fullListNames))
print('\n', fullListNames)
print( '\n',  len(listNames))
print('\n', listNames)