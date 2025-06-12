#import pathlib
import re
from pathlib import Path
import operator

mainFile = "main.cpp"
sourceDirectories = {"simulation", "geneticalgorithm"}
excludes = {"geneticalgorithm/Visualizer.h", "geneticalgorithm/Visualizer.cpp", "geneticalgorithm/utildraw.h"}

disabledLibraries = {"#include <raylib.h>\n"}

sourceFiles = [
"simulation/Float.h",
"simulation/Pt2f.h",
"simulation/Constants.h",
"simulation/Collision.h",
"simulation/Land.h",
"simulation/ShipControl.h",
"simulation/Simulation.h",
"simulation/Collision.cpp",
"simulation/Land.cpp",
"simulation/Pt2f.cpp",
"simulation/ShipControl.cpp",
"simulation/Simulation.cpp",
"geneticalgorithm/Random.h",
"geneticalgorithm/Util.h",
"geneticalgorithm/Parameters.h",
"geneticalgorithm/Chromosome.h",
"geneticalgorithm/DoubleBuffer.h",
"geneticalgorithm/SimulationPlayer.h",
"geneticalgorithm/GeneticAlgorithmTrace.h",
"geneticalgorithm/GeneticAlgorithm.h",
#"geneticalgorithm/utildraw.h",
"geneticalgorithm/Visualizer.h",
"geneticalgorithm/Chromosome.cpp",
"geneticalgorithm/GeneticAlgorithm.cpp",
"geneticalgorithm/GeneticAlgorithmTrace.cpp",
"geneticalgorithm/SimulationPlayer.cpp",
"geneticalgorithm/Util.cpp",
"geneticalgorithm/Visualizer.cpp",
"main.cpp"
]

# def findFiles():
#     fileList = []
#     for sourceDirectory in sourceDirectories:
#         for filename in Path(sourceDirectory).iterdir():
#             filenamestr = str(filename)
#             if(not excludes.intersection({filenamestr})):
#                 fileList.append(filename)
#                 print("Adding", filenamestr)
#             else:
#                 print("Excluding",  filenamestr)
#     return  fileList

# def findFilesSorted():
#     fileList = findFiles()
#     fileList.sort(key= operator.attrgetter("suffix"))
#     fileList.append(Path(mainFile))
#     return fileList


def addFileToOutput(filename):
    out = []
    systemincludes = set()
    isCpp = Path(filename).suffix == ".cpp"
    with open(filename, 'r') as f:
        out.append("\n// ------- " + filename + " -------\n")
        for line in f.readlines():
            # Check if system include
            if not (re.match("^#include <", line) is None):
                systemincludes.add(line)
                print("Added system include :", filename, ":", line, end='')
            # Check if user include or pragma
            elif not (re.match("#(pragma|include [^<])",line) is None):
                print("Ignoring line: ", filename, ":", line, end='')
            # Add to output otherwise
            else:
               out.append(line)

    out.append("\n")
    return (out,systemincludes)

def generateOutput(outFile):
    fileList = sourceFiles
    output = []
    systemincludes = set()

    for fs in fileList:
        o,si = addFileToOutput(str(fs))
        for oi in o: output.append(oi)
        systemincludes.update(si)

    output.insert(0,"")
    
    tempsystemincludes = systemincludes.difference(disabledLibraries)
    print("Removed system libraries: " , systemincludes.intersection(disabledLibraries))
    systemincludes = tempsystemincludes

    for si in systemincludes:
        output.insert(0,si)
    output.insert(0, "\n// ------ System include ------\n")

    print("Writing to ", outFile)
    with open(outFile,'w') as outf:
        outf.writelines(output)

if __name__ == "__main__":
    print("Entered main")
    Path("export").mkdir(exist_ok=True)
    generateOutput("export/export.cpp")