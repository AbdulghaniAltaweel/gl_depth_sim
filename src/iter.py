# -*- coding: utf-8 -*-
#%%
import os
from os.path import splitext

myExtension1 = ".obj"
myExtension2 = ".dae"
myExtension3 = ".stl"
myPath = "/home/ai/Downloads/testDataGen"

#create an new List to include the Names of all object-mesh files
fullListNames = []
listNames = []
fullListNames = os.listdir(myPath)
print('\n', fullListNames)

#%%
#function to extract the extension and the file names. When Required
from os.path import splitext
def splitext_(path):
    if len(path.split('.')) > 2:
        return path.split('.')[0],'.'.join(path.split('.')[-2:])
    return splitext(path)

#%%
#funtion to list only the required files. Ignoreing files with unwanted Extention
for file_ in fullListNames:
    file_name,extension = splitext_(file_)
    if extension == myExtension1 or myExtension2 or myExtension3:
        listNames.append(file_name)
print('\n',listNames)

#%%
for file_ in fullListNames:
    file_name,extension = splitext_(file_)
    if extension == myExtension1 or myExtension2 or myExtension3:
        print (file_name)

print( '\n',  len(fullListNames))